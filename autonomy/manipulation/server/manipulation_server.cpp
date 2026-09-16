/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/server/manipulation_server.hpp"

#include <filesystem>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autonomy/manipulation/execution/autolink_trajectory_controller.hpp"
#include "autonomy/manipulation/execution/simple_controller_manager.hpp"
#include "autonomy/manipulation/collision/aabb_collision_detector.hpp"
#include "autonomy/manipulation/collision/collision_detector.hpp"
#include "autonomy/manipulation/core/link_fk.hpp"
#include "autonomy/manipulation/core/metrics.hpp"
#include "autonomy/manipulation/kinematics/cached_kinematics.hpp"
#include "autonomy/manipulation/planning/joint_interpolation_planner.hpp"
#include "autonomy/manipulation/plugins.hpp"
#include "autonomy/manipulation/scene/simple_planning_scene.hpp"
#include "autonomy/manipulation/server/manipulation_action_server.hpp"

#ifdef AUTONOMY_HAS_KDL
#include "autonomy/manipulation/kinematics/kdl_kinematics.hpp"
#endif

namespace autonomy {
namespace manipulation {

ManipulationServer::~ManipulationServer() {
  Stop();
}

bool ManipulationServer::ResolveUrdfPath(std::string* urdf_path) const {
  if (!urdf_path || urdf_path->empty()) {
    return true;
  }
  if (urdf_path->front() == '/' || urdf_path->rfind("file://", 0) == 0) {
    return true;
  }
  const std::string work = common::AutonomyWorkRoot();
  std::vector<std::string> candidates = {*urdf_path, work + "/" + *urdf_path};
  constexpr char kSharePrefix[] = "share/autonomy/";
  if (urdf_path->rfind(kSharePrefix, 0) == 0) {
    candidates.push_back(work + "/autonomy/" +
                         urdf_path->substr(sizeof(kSharePrefix) - 1));
  }
  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      *urdf_path = candidate;
      return true;
    }
  }
  return true;
}

bool ManipulationServer::SetupPlugins(const std::string& urdf_path,
                                      const std::string& group) {
  RegisterManipulationPlugins();

  const std::string kin_id = options_.kinematics_solver().empty()
                                 ? "stub"
                                 : options_.kinematics_solver();
  kinematics_ = CreatePlugin<kinematics::KinematicsBase>(kin_id);
  if (!kinematics_) {
    AERROR << "Unknown kinematics_solver=" << kin_id;
    return false;
  }
  if (!kinematics_->Init(group, options_.base_frame(), options_.tip_frame())) {
    return false;
  }
#ifdef AUTONOMY_HAS_KDL
  auto load_kdl = [&](kinematics::KdlKinematics* kdl) -> bool {
    if (!kdl) {
      return true;
    }
    if (!urdf_path.empty() && !kdl->LoadUrdf(urdf_path)) {
      return false;
    }
    if (!kdl->JointNames().empty()) {
      model_->SetGroupJoints(group, kdl->JointNames());
    }
    return true;
  };
  if (!load_kdl(
          dynamic_cast<kinematics::KdlKinematics*>(kinematics_.get()))) {
    return false;
  }
  if (auto* cached =
          dynamic_cast<kinematics::CachedKinematics*>(kinematics_.get())) {
    if (!load_kdl(dynamic_cast<kinematics::KdlKinematics*>(
            cached->Inner().get()))) {
      return false;
    }
  }
#endif

  const std::string planner_id = options_.planner_id().empty()
                                     ? "joint_interpolation"
                                     : options_.planner_id();
  planner_ = CreatePlugin<planning::PlannerBase>(planner_id);
  if (!planner_) {
    AERROR << "Unknown planner_id=" << planner_id;
    return false;
  }
  if (!planner_->Init(planner_id)) {
    return false;
  }
  if (auto* ji =
          dynamic_cast<planning::JointInterpolationPlanner*>(planner_.get())) {
    if (options_.interpolation_steps() > 0) {
      ji->SetNumSteps(static_cast<int>(options_.interpolation_steps()));
    }
  }

  const std::string col_id =
      options_.collision_detector().empty() ? "aabb"
                                            : options_.collision_detector();
  auto detector = CreatePlugin<collision::CollisionDetector>(col_id);
  if (!detector) {
    detector = CreatePlugin<collision::CollisionDetector>("aabb");
  }
  if (detector) {
    detector->Init(col_id);
    if (auto* aabb =
            dynamic_cast<collision::AabbCollisionDetector*>(detector.get())) {
      if (state_) {
        aabb->SetLinkTree(
            std::make_shared<core::LinkFkTree>(state_->LinkTree()));
      }
    }
    scene_->SetCollisionDetector(detector);
  }

  execution_ = std::make_unique<execution::TrajectoryExecutionManager>();
  if (options_.execution_timeout_s() > 0) {
    execution_->SetTimeout(options_.execution_timeout_s());
  }

  const std::string ctrl_id = options_.controller_id().empty()
                                  ? "arm_controller"
                                  : options_.controller_id();
  std::shared_ptr<execution::ControllerManager> controller;
  if (options_.use_autolink_trajectory()) {
    auto c = std::make_shared<execution::AutolinkTrajectoryController>();
    const std::string traj_topic = options_.trajectory_topic().empty()
                                       ? kJointTrajectoryTopic
                                       : options_.trajectory_topic();
    c->SetTopic(traj_topic);
    c->SetNode(node_);
    c->Init(ctrl_id);
    controller = c;
  } else {
    controller = CreatePlugin<execution::ControllerManager>("simple");
    if (controller) {
      controller->Init(ctrl_id);
    }
  }
  if (controller) {
    execution_->RegisterController(ctrl_id, controller);
    execution_->SetActiveController(ctrl_id);
  }
  if (!options_.gripper_controller_id().empty()) {
    auto grip = CreatePlugin<execution::ControllerManager>("simple");
    if (grip && grip->Init(options_.gripper_controller_id())) {
      execution_->RegisterController(options_.gripper_controller_id(), grip);
    }
  }

  return true;
}

bool ManipulationServer::Init(const proto::ManipulationOptions& options) {
  options_ = options;
  node_ = autolink::CreateNode("manipulation");
  if (!node_) {
    AERROR << "ManipulationServer: CreateNode failed";
    return false;
  }

  model_ = std::make_shared<core::SimpleRobotModel>();
  std::string urdf_path = options_.robot_description();
  ResolveUrdfPath(&urdf_path);
  urdf_path_ = urdf_path;
  if (!model_->Load(urdf_path, options_.robot_description_semantic())) {
    AERROR << "ManipulationServer: robot model load failed";
    return false;
  }

  const std::string group = options_.planning_group().empty()
                                ? "arm"
                                : options_.planning_group();
  // Prefer SRDF group joints; only fall back when group is missing.
  if (!model_->HasGroup(group) || model_->GetJointNames(group).empty()) {
    if (!model_->AllJoints().empty()) {
      model_->SetGroupJoints(group, model_->AllJoints());
    } else if (!options_.default_joints().empty()) {
      std::vector<std::string> joints(options_.default_joints().begin(),
                                      options_.default_joints().end());
      model_->SetGroupJoints(group, std::move(joints));
    }
  }
  model_->SetGroupMeta(group, options_.base_frame(), options_.tip_frame());

  state_ = std::make_shared<core::SimpleRobotState>(model_);
  if (!urdf_path.empty()) {
    state_->LoadLinkTree(urdf_path);
  }
  scene_ = std::make_shared<scene::SimplePlanningScene>();
  if (state_) {
    scene_->SetLinkTree(
        std::make_shared<core::LinkFkTree>(state_->LinkTree()));
  }
  for (const auto& pair : model_->DisabledCollisions()) {
    scene_->SetAllowedCollision(pair.first, pair.second, true);
  }
  scene_monitor_ = std::make_shared<scene::SceneMonitor>();
  scene_monitor_->SetScene(scene_);

  if (!SetupPlugins(urdf_path, group)) {
    return false;
  }

  joint_states_ = std::make_shared<execution::JointStateSubscriber>();
  joint_states_->Init(node_, options_.joint_states_topic());
  if (options_.enable_scene_monitor()) {
    scene_monitor_->SetJointStateSubscriber(joint_states_);
  }

  pipeline_ = std::make_unique<planning::PlanningPipeline>();
  if (!pipeline_->Init(options_)) {
    return false;
  }
  pipeline_->SetPlanner(planner_);

  auto add_cap = [this](const std::string& id) {
    auto cap = CreatePlugin<server::Capability>(id);
    if (!cap || !cap->Init(this)) {
      return false;
    }
    capabilities_[cap->Name()] = std::move(cap);
    return true;
  };
  RegisterManipulationPlugins();
  if (!add_cap("plan") || !add_cap("execute") || !add_cap("plan_and_execute") ||
      !add_cap("cartesian_path") || !add_cap("fk_ik") ||
      !add_cap("get_planning_scene") || !add_cap("apply_planning_scene") ||
      !add_cap("state_validation") || !add_cap("query_planners") ||
      !add_cap("clear_octomap") || !add_cap("clear_scene") ||
      !add_cap("validate_trajectory") || !add_cap("get_urdf") ||
      !add_cap("save_load_geometry")) {
    AERROR << "ManipulationServer: capability init failed";
    return false;
  }

  if (execution_ && joint_states_) {
    execution_->SetStateProvider(
        [this]() { return joint_states_->Latest(); });
    if (options_.execution_deviation_tol() > 0) {
      execution_->SetDeviationTolerance(options_.execution_deviation_tol());
    }
    execution_->SetDeviationHook(
        [](const core::JointState& desired, const core::JointState& actual) {
          AWARN << "TrajectoryExecutionManager deviation desired_dof="
                << desired.positions.size()
                << " actual_dof=" << actual.positions.size();
        });
  }

  if (options_.enable_servo()) {
    servo_ = std::make_shared<servo::DampedLeastSquaresServo>();
    servo_->SetKinematics(kinematics_);
    servo_->SetScene(scene_);
    servo_->SetModel(model_);
    core::JointState zero;
    zero.names = model_->GetJointNames(group);
    zero.positions.assign(zero.names.size(), 0.0);
    servo_->SetState(zero);
    servo_->Init();
    servo_node_ = std::make_unique<servo::ServoNode>();
    servo_node_->SetServo(servo_);
  }

  action_server_ = std::make_unique<ManipulationActionServer>(this);
  const std::string action_name = options_.action_name().empty()
                                      ? kManipulationAction
                                      : options_.action_name();
  if (!action_server_->Init(node_, action_name)) {
    AERROR << "ManipulationServer: action server init failed";
    return false;
  }

  AINFO << "ManipulationServer ready group=" << group
        << " kinematics=" << options_.kinematics_solver()
        << " planner=" << options_.planner_id();
  return true;
}

bool ManipulationServer::Start() {
  running_ = true;
  if (servo_node_) {
    servo_node_->Start();
  }
  return true;
}

void ManipulationServer::Stop() {
  if (servo_node_) {
    servo_node_->Stop();
  }
  if (action_server_) {
    action_server_->Shutdown();
  }
  if (execution_) {
    execution_->Cancel();
  }
  running_ = false;
}

planning::MotionPlanResponse ManipulationServer::Plan(
    const planning::MotionPlanRequest& request) {
  metrics::ManipulationMetrics::Instance().OnPlanStart();
  planning::MotionPlanRequest req = request;
  if (req.group.empty()) {
    req.group = options_.planning_group();
  }
  if (req.planner_id.empty()) {
    req.planner_id = options_.planner_id();
  }
  if (req.planning_time <= 0.0 && options_.planning_time_ms() > 0) {
    req.planning_time =
        static_cast<double>(options_.planning_time_ms()) / 1000.0;
  }
  if (req.max_attempts <= 0 && options_.max_planning_attempts() > 0) {
    req.max_attempts = static_cast<int>(options_.max_planning_attempts());
  }
  if (options_.max_velocity() > 0) {
    req.max_velocity = options_.max_velocity();
  }
  if (options_.max_acceleration() > 0) {
    req.max_acceleration = options_.max_acceleration();
  }
  req.scene = scene_;
  req.kinematics = kinematics_;
  req.model = model_;

  if (req.start_state.positions.empty() && scene_) {
    req.start_state = scene_->GetCurrentState();
  }
  if (req.start_state.positions.empty()) {
    const auto names =
        model_ ? model_->GetJointNames(req.group) : std::vector<std::string>{};
    if (!names.empty()) {
      req.start_state.names = names;
      req.start_state.positions.assign(names.size(), 0.0);
    }
  }

  if (req.goal_state.names.empty() && !req.goal_state.positions.empty() &&
      model_) {
    req.goal_state.names = model_->GetJointNames(req.group);
  }

  // Switch planner if request asks for a different id.
  if (!req.planner_id.empty() && planner_ &&
      req.planner_id != options_.planner_id()) {
    auto alt = CreatePlugin<planning::PlannerBase>(req.planner_id);
    if (alt && alt->Init(req.planner_id)) {
      pipeline_->SetPlanner(alt);
    }
  } else {
    pipeline_->SetPlanner(planner_);
  }

  if (req.has_goal_pose && kinematics_ &&
      req.planner_id != "cartesian") {
    core::JointState seed = req.start_state;
    core::JointState ik;
    const auto code = kinematics_->GetPositionIK(req.goal_pose, seed, {}, &ik);
    metrics::ManipulationMetrics::Instance().OnIk(code == ErrorCode::kSuccess);
    if (code != ErrorCode::kSuccess) {
      planning::MotionPlanResponse failed;
      failed.error = "IK failed";
      failed.error_code = code;
      metrics::ManipulationMetrics::Instance().OnPlanEnd(false);
      return failed;
    }
    req.goal_state = ik;
    req.has_goal_pose = false;
  }

  if (!pipeline_) {
    planning::MotionPlanResponse failed;
    failed.error = "no pipeline";
    failed.error_code = ErrorCode::kFailure;
    metrics::ManipulationMetrics::Instance().OnPlanEnd(false);
    return failed;
  }
  auto response = pipeline_->Plan(req);
  if (response.success && scene_ && !response.trajectory.waypoints.empty()) {
    scene_->SetCurrentState(response.trajectory.waypoints.back());
  }
  metrics::ManipulationMetrics::Instance().OnPlanEnd(response.success);
  return response;
}

ErrorCode ManipulationServer::ExecuteTrajectory(
    const core::RobotTrajectory& trajectory, bool replace) {
  if (!execution_) {
    return ErrorCode::kControlFailed;
  }
  return execution_->Execute(trajectory, replace);
}

bool ManipulationServer::Execute(const core::RobotTrajectory& trajectory,
                                 bool replace) {
  return ExecuteTrajectory(trajectory, replace) == ErrorCode::kSuccess;
}

void ManipulationServer::CancelExecution() {
  if (execution_) {
    execution_->Cancel();
  }
}

planning::PlanningPipeline* ManipulationServer::pipeline() {
  return pipeline_.get();
}

scene::PlanningScene* ManipulationServer::scene() {
  return scene_.get();
}

core::SimpleRobotModel* ManipulationServer::model() {
  return model_.get();
}

kinematics::KinematicsBase* ManipulationServer::kinematics() {
  return kinematics_.get();
}

scene::SceneMonitor* ManipulationServer::scene_monitor() {
  return scene_monitor_.get();
}

execution::TrajectoryExecutionManager*
ManipulationServer::execution_manager() {
  return execution_.get();
}

server::Capability* ManipulationServer::GetCapability(
    const std::string& name) {
  const auto it = capabilities_.find(name);
  return it == capabilities_.end() ? nullptr : it->second.get();
}

bool ManipulationServer::SetPlannerParams(
    const std::string& planner_id,
    const std::unordered_map<std::string, std::string>& params) {
  bool changed = false;
  if (!planner_id.empty()) {
    options_.set_planner_id(planner_id);
    changed = true;
  }
  for (const auto& kv : params) {
    if (kv.first == "planner_id" && !kv.second.empty()) {
      options_.set_planner_id(kv.second);
      changed = true;
    } else if (kv.first == "planning_time_ms") {
      options_.set_planning_time_ms(
          static_cast<uint32_t>(std::stoul(kv.second)));
      changed = true;
    } else if (kv.first == "max_velocity") {
      options_.set_max_velocity(std::stod(kv.second));
      changed = true;
    } else if (kv.first == "max_acceleration") {
      options_.set_max_acceleration(std::stod(kv.second));
      changed = true;
    } else if (kv.first == "interpolation_steps") {
      options_.set_interpolation_steps(
          static_cast<uint32_t>(std::stoul(kv.second)));
      changed = true;
    } else if (kv.first == "max_attempts") {
      options_.set_max_planning_attempts(
          static_cast<uint32_t>(std::stoul(kv.second)));
      changed = true;
    }
  }
  return changed;
}

}  // namespace manipulation
}  // namespace autonomy
