/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/manipulation_server.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autonomy/manipulation/motion/execution/joint_trajectory_controller.hpp"
#include "autonomy/manipulation/motion/execution/effort_tracking_controller.hpp"
#include "autonomy/manipulation/motion/execution/logging_trajectory_controller.hpp"
#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/manipulation_metrics.hpp"
#include "autonomy/manipulation/motion/kinematics/cached_kinematics.hpp"
#include "autonomy/manipulation/plugin_ids.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"
#include "autonomy/manipulation/dispatch/manipulation_action_server.hpp"

#include <automsgs/msgs/control_msgs/joint_jog.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/std_msgs/float64_multi_array.pb.h>
#include <automsgs/msgs/std_msgs/int32.pb.h>

#ifdef AUTONOMY_HAS_KDL
#include "autonomy/manipulation/motion/kinematics/kdl_kinematics.hpp"
#include "autonomy/manipulation/motion/kinematics/trac_ik_kinematics.hpp"
#endif

namespace autonomy {
namespace manipulation {

ManipulationServer::ManipulationServer() = default;

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
  kinematics_ = CreatePlugin<common::KinematicsInterface>(kin_id);
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
            cached->GetUnderlyingSolver().get()))) {
      return false;
    }
  }
  if (auto* trac =
          dynamic_cast<kinematics::TracIkKinematics*>(kinematics_.get())) {
    if (!urdf_path.empty() && !trac->LoadUrdf(urdf_path)) {
      return false;
    }
  }
#endif

  const std::string planner_id =
      options_.planner_id().empty() ? "pilz_ptp" : options_.planner_id();
  planner_ = CreatePlugin<common::PlannerInterface>(planner_id);
  if (!planner_) {
    AERROR << "Unknown planner_id=" << planner_id;
    return false;
  }
  if (!planner_->Init(planner_id)) {
    return false;
  }

  dynamics_ = dynamics::CreateDynamicsSolver(urdf_path);
  if (dynamics::HasPinocchioDynamics()) {
    AINFO << "ManipulationServer: Pinocchio dynamics enabled";
  }

  const std::string col_id =
      options_.collision_detector().empty() ? "aabb"
                                            : options_.collision_detector();
  auto detector = CreatePlugin<common::CollisionInterface>(col_id);
  if (!detector) {
    detector = CreatePlugin<common::CollisionInterface>("aabb");
  }
  if (detector) {
    detector->Init(col_id);
    if (state_) {
      detector->SetLinkTree(
          std::make_shared<model::LinkForwardKinematicsTree>(state_->GetLinkForwardKinematicsTree()));
    }
    if (!urdf_path.empty()) {
      auto shapes = std::make_shared<collision::LinkCollisionModel>();
      std::string shape_err;
      if (shapes->LoadFromUrdf(urdf_path, &shape_err)) {
        detector->SetLinkCollisionModel(shapes);
      } else if (!shape_err.empty()) {
        AWARN << "LinkCollisionModel: " << shape_err;
      }
    }
    detector->SetPadding(0.01);  // MoveIt-style default padding
    scene_->SetCollisionDetector(detector);
  }

  execution_ = std::make_unique<execution::TrajectoryExecutionManager>();
  if (options_.execution_timeout_seconds() > 0) {
    execution_->SetTimeout(options_.execution_timeout_seconds());
  }

  const std::string ctrl_id = options_.controller_id().empty()
                                  ? "arm_controller"
                                  : options_.controller_id();
  common::ControllerInterface::SharedPtr controller;
  if (options_.use_joint_trajectory_controller()) {
    auto c = std::make_shared<execution::JointTrajectoryController>();
    const std::string traj_topic = options_.trajectory_topic().empty()
                                       ? kJointTrajectoryTopic
                                       : options_.trajectory_topic();
    c->SetTopic(traj_topic);
    c->SetNode(node_);
    c->Init(ctrl_id);
    controller = c;
  } else {
    controller = CreatePlugin<common::ControllerInterface>("simple");
    if (controller) {
      controller->Init(ctrl_id);
    }
  }
  if (controller) {
    execution_->RegisterController(ctrl_id, controller);
    execution_->SetActiveController(ctrl_id);
  }
  if (!options_.gripper_controller_id().empty()) {
    auto grip = CreatePlugin<common::ControllerInterface>("simple");
    if (grip && grip->Init(options_.gripper_controller_id())) {
      execution_->RegisterController(options_.gripper_controller_id(), grip);
    }
  }

  return true;
}

bool ManipulationServer::Init(const ManipulationOptions& options) {
  options_ = options;
  node_ = autolink::CreateNode("manipulation");
  if (!node_) {
    AERROR << "ManipulationServer: CreateNode failed";
    return false;
  }

  model_ = std::make_shared<model::SimpleRobotModel>();
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
  model_->SetGroupChainFrames(group, options_.base_frame(), options_.tip_frame());

  state_ = std::make_shared<model::SimpleRobotState>(model_);
  if (!urdf_path.empty()) {
    state_->LoadLinkForwardKinematicsTree(urdf_path);
  }
  scene_ = std::make_shared<scene::SimplePlanningScene>();
  if (state_) {
    scene_->SetLinkTree(
        std::make_shared<model::LinkForwardKinematicsTree>(state_->GetLinkForwardKinematicsTree()));
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

  pipeline_ = std::make_unique<planner::PlanningPipeline>();
  if (!pipeline_->Init(options_)) {
    return false;
  }
  pipeline_->SetPlanner(planner_);

  auto add_cap = [this](const std::string& id) {
    auto cap = CreatePlugin<dispatch::Capability>(id);
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
      !add_cap("save_load_geometry") || !add_cap("get_dynamics")) {
    AERROR << "ManipulationServer: capability init failed";
    return false;
  }

  if (execution_ && joint_states_) {
    execution_->SetStateProvider(
        [this]() { return joint_states_->GetLatestJointState(); });
    if (options_.execution_deviation_tolerance() > 0) {
      execution_->SetDeviationTolerance(options_.execution_deviation_tolerance());
    }
    if (options_.execution_effort_tolerance() > 0) {
      execution_->SetEffortTrackingTolerance(options_.execution_effort_tolerance());
    }
    execution_->SetDeviationHook(
        [](const automsgs::msgs::sensor_msgs::JointState& desired, const automsgs::msgs::sensor_msgs::JointState& actual) {
          AWARN << "TrajectoryExecutionManager deviation desired_dof="
                << desired.position_size()
                << " actual_dof=" << actual.position_size()
                << " desired_effort=" << desired.effort_size()
                << " actual_effort=" << actual.effort_size();
        });
    if (scene_) {
      execution_->SetSceneValidityChecker([this](const automsgs::msgs::sensor_msgs::JointState& q) {
        return scene_ && scene_->IsStateValid(q);
      });
    }
    execution_->SetEffortFeedforwardHook(
        [this](const automsgs::msgs::sensor_msgs::JointState& commanded) {
          double l1 = 0.0;
          for (double e : commanded.effort()) {
            l1 += std::abs(e);
          }
          AINFO << "effort feedforward joints=" << commanded.effort_size()
                << " l1=" << l1;
          if (!effort_tracker_) {
            return;
          }
          effort_tracker_->SetDesired(commanded);
          effort_tracker_->Publish();
        });
    if (node_) {
      effort_command_writer_ =
          node_->CreateWriter<automsgs::msgs::std_msgs::Float64MultiArray>(
              kEffortCommandTopic);
      if (effort_command_writer_) {
        AINFO << "Effort command Float64MultiArray on " << kEffortCommandTopic;
      }
      effort_tracker_ = std::make_unique<execution::EffortTrackingController>();
      effort_tracker_->SetProportionalGain(options_.effort_tracking_proportional_gain());
      effort_tracker_->SetMaxAbsEffort(options_.maximum_effort_command());
      effort_tracker_->SetStateProvider(
          [this]() { return joint_states_->GetLatestJointState(); });
      effort_tracker_->SetCommandPublisher(
          [this](const std::vector<double>& cmd) {
            if (!effort_command_writer_) {
              return;
            }
            automsgs::msgs::std_msgs::Float64MultiArray msg;
            for (double e : cmd) {
              msg.add_data(e);
            }
            effort_command_writer_->Write(msg);
          });
      AINFO << "EffortTrackingController kp=" << options_.effort_tracking_proportional_gain()
            << " max=" << options_.maximum_effort_command();
    }
  }

  if (options_.enable_servo()) {
    servo_ = std::make_shared<servo::DampedLeastSquaresCartesianServo>();
    servo_->SetKinematics(kinematics_);
    servo_->SetScene(scene_);
    servo_->SetModel(model_);
    automsgs::msgs::sensor_msgs::JointState zero;
    const auto joint_names = model_->GetJointNames(group);
    SetJointState(&zero, joint_names,
                  std::vector<double>(joint_names.size(), 0.0));
    servo_->SetState(zero);
    servo_->Init();
    servo_node_ = std::make_unique<servo::CartesianServoNode>();
    servo_node_->SetServo(servo_);
    servo_publisher_ =
        std::make_shared<execution::JointTrajectoryController>();
    servo_publisher_->SetNode(node_);
    servo_publisher_->SetTopic(kServoJointCommandTopic);
    servo_publisher_->Init("servo");
    servo_node_->SetCommandPublisher(
        [this](const automsgs::msgs::sensor_msgs::JointState& cmd) {
          if (!servo_publisher_) {
            return;
          }
          automsgs::msgs::trajectory_msgs::JointTrajectory traj;
          AddTrajectoryPoint(&traj, cmd, 0.01);
          servo_publisher_->Execute(traj);
        });
    if (joint_states_) {
      joint_states_->SetCallback([this](const automsgs::msgs::sensor_msgs::JointState& s) {
        if (servo_) {
          servo_->SetState(s);
        }
      });
    }
    if (node_) {
      servo_twist_reader_ =
          node_->CreateReader<automsgs::msgs::geometry_msgs::TwistStamped>(
              kServoTwistCommandTopic,
              [this](const std::shared_ptr<
                         automsgs::msgs::geometry_msgs::TwistStamped>& msg) {
                if (!msg || !servo_node_) {
                  return;
                }
                servo_node_->SetCommand(msg->twist());
              });
      if (servo_twist_reader_) {
        AINFO << "CartesianServo automsgs::msgs::geometry_msgs::TwistStamped on " << kServoTwistCommandTopic;
      }

      servo_pose_reader_ =
          node_->CreateReader<automsgs::msgs::geometry_msgs::PoseStamped>(
              kServoPoseCommandTopic,
              [this](const std::shared_ptr<
                         automsgs::msgs::geometry_msgs::PoseStamped>& msg) {
                if (!msg || !servo_node_) {
                  return;
                }
                servo_node_->SetPose(*msg);
              });
      if (servo_pose_reader_) {
        AINFO << "CartesianServo automsgs::msgs::geometry_msgs::PoseStamped on " << kServoPoseCommandTopic;
      }

      servo_jog_reader_ =
          node_->CreateReader<automsgs::msgs::control_msgs::JointJog>(
              kServoJointJogCommandTopic,
              [this](const std::shared_ptr<
                         automsgs::msgs::control_msgs::JointJog>& msg) {
                if (!msg || !servo_node_) {
                  return;
                }
                automsgs::msgs::control_msgs::JointJog jog = *msg;
                // Displacements (m/rad) treated as one-shot Δq / dt when
                // velocities empty (MoveIt automsgs::msgs::control_msgs::JointJog often fills one of them).
                if (jog.velocities_size() == 0 &&
                    jog.displacements_size() > 0) {
                  const double dt =
                      jog.duration() > 1e-6 ? jog.duration() : 0.01;
                  jog.clear_velocities();
                  for (double d : jog.displacements()) {
                    jog.add_velocities(d / dt);
                  }
                }
                servo_node_->SetJointJog(jog);
              });
      if (servo_jog_reader_) {
        AINFO << "CartesianServo automsgs::msgs::control_msgs::JointJog on " << kServoJointJogCommandTopic;
      }

      servo_cmd_type_reader_ =
          node_->CreateReader<automsgs::msgs::std_msgs::Int32>(
              kServoCommandTypeTopic,
              [this](const std::shared_ptr<automsgs::msgs::std_msgs::Int32>&
                         msg) {
                if (!msg || !servo_node_) {
                  return;
                }
                switch (msg->data()) {
                  case 1:
                    servo_node_->SetCommandType(servo::CommandType::kJointJog);
                    break;
                  case 2:
                    servo_node_->SetCommandType(servo::CommandType::kPose);
                    break;
                  case 0:
                  default:
                    servo_node_->SetCommandType(servo::CommandType::kTwist);
                    break;
                }
              });
      if (servo_cmd_type_reader_) {
        AINFO << "CartesianServo CommandType Int32 on " << kServoCommandTypeTopic;
      }

      servo_status_writer_ =
          node_->CreateWriter<automsgs::msgs::std_msgs::Int32>(
              kServoStatusTopic);
      if (servo_status_writer_) {
        servo_node_->SetStatusPublisher(
            [this](servo::CartesianServoStatus st) {
              if (!servo_status_writer_) {
                return;
              }
              automsgs::msgs::std_msgs::Int32 msg;
              msg.set_data(static_cast<int32_t>(st));
              servo_status_writer_->Write(msg);
            });
        AINFO << "CartesianServo status Int32 on " << kServoStatusTopic;
      }
    }
  }

  if (options_.enable_scene_monitor() && scene_monitor_ && node_) {
    occupancy_monitor_ = std::make_unique<scene::OccupancyMapMonitor>();
    occupancy_monitor_->SetSceneMonitor(scene_monitor_);
    occupancy_monitor_->SetRobotModel(model_);
    if (options_.occupancy_self_filter_padding() > 0) {
      occupancy_monitor_->SetSelfFilterPadding(
          options_.occupancy_self_filter_padding());
    }
    occupancy_monitor_->Start(node_, kPointCloudTopic, kOctomapTopic);
  }

  action_server_ = std::make_unique<dispatch::ManipulationActionServer>(this);
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
  running_ = false;
  if (servo_node_) {
    servo_node_->Stop();
  }
  servo_twist_reader_.reset();
  servo_pose_reader_.reset();
  servo_jog_reader_.reset();
  servo_cmd_type_reader_.reset();
  servo_status_writer_.reset();
  effort_command_writer_.reset();
  if (effort_tracker_) {
    effort_tracker_->Clear();
  }
  effort_tracker_.reset();
  if (occupancy_monitor_) {
    occupancy_monitor_->Stop();
  }
  if (action_server_) {
    action_server_->Shutdown();
  }
  if (execution_) {
    execution_->Cancel();
  }
}

::autonomy::manipulation::proto::MotionPlanResponse ManipulationServer::Plan(
    const planner::MotionPlanRequest& request) {
  metrics::ManipulationMetrics::Instance().RecordPlanAttemptStart();
  planner::MotionPlanRequest req = request;
  if (req.pb.group().empty()) {
    req.pb.set_group(options_.planning_group());
  }
  if (req.pb.planner_id().empty()) {
    req.pb.set_planner_id(options_.planner_id());
  }
  if (req.pb.planning_time() <= 0.0 && options_.planning_time_milliseconds() > 0) {
    req.pb.set_planning_time(static_cast<double>(options_.planning_time_milliseconds()) / 1000.0);
  }
  if (req.pb.max_attempts() <= 0 && options_.max_planning_attempts() > 0) {
    req.pb.set_max_attempts(static_cast<int>(options_.max_planning_attempts()));
  }
  if (options_.max_velocity() > 0) {
    req.pb.set_max_velocity(options_.max_velocity());
  }
  if (options_.max_acceleration() > 0) {
    req.pb.set_max_acceleration(options_.max_acceleration());
  }
  if (options_.goal_joint_tolerance() > 0) {
    req.pb.set_goal_joint_tolerance(options_.goal_joint_tolerance());
  }
  if (options_.goal_position_tolerance() > 0) {
    req.pb.set_goal_position_tolerance(options_.goal_position_tolerance());
  }
  if (options_.max_translational_velocity() > 0 || options_.max_rotational_velocity() > 0) {
    req.pb.mutable_cartesian_limits()->set_configured(true);
    if (options_.max_translational_velocity() > 0) {
      req.pb.mutable_cartesian_limits()->set_max_translational_velocity(options_.max_translational_velocity());
    }
    if (options_.max_translational_acceleration() > 0) {
      req.pb.mutable_cartesian_limits()->set_max_translational_acceleration(options_.max_translational_acceleration());
    }
    if (options_.max_translational_deceleration() > 0) {
      req.pb.mutable_cartesian_limits()->set_max_translational_deceleration(options_.max_translational_deceleration());
    } else if (options_.max_translational_acceleration() > 0) {
      req.pb.mutable_cartesian_limits()->set_max_translational_deceleration(options_.max_translational_acceleration());
    }
    if (options_.max_rotational_velocity() > 0) {
      req.pb.mutable_cartesian_limits()->set_max_rotational_velocity(options_.max_rotational_velocity());
    }
  }
  req.scene = scene_;
  req.kinematics = kinematics_;
  req.model = model_;
  if (state_) {
    req.link_tree = std::make_shared<model::LinkForwardKinematicsTree>(state_->GetLinkForwardKinematicsTree());
  } else if (model_ && !model_->GetLinkForwardKinematicsTree().LinkNames().empty()) {
    req.link_tree = std::make_shared<model::LinkForwardKinematicsTree>(model_->GetLinkForwardKinematicsTree());
  }

  if (req.pb.start_state().position_size() == 0 && scene_) {
    *req.pb.mutable_start_state() = scene_->GetCurrentState();
  }
  if (req.pb.start_state().position_size() == 0) {
    const auto names =
        model_ ? model_->GetJointNames(req.pb.group()) : std::vector<std::string>{};
    if (!names.empty()) {
      SetJointState(req.pb.mutable_start_state(), names,
                    std::vector<double>(names.size(), 0.0));
    }
  }

  if (req.pb.goal_state().name_size() == 0 && req.pb.goal_state().position_size() > 0 &&
      model_) {
    req.pb.mutable_goal_state()->clear_name();
    for (const auto& n : model_->GetJointNames(req.pb.group())) {
      req.pb.mutable_goal_state()->add_name(n);
    }
  }

  // Switch planner if request asks for a different id.
  if (!req.pb.planner_id().empty() && planner_ &&
      req.pb.planner_id() != options_.planner_id()) {
    auto alt = CreatePlugin<common::PlannerInterface>(req.pb.planner_id());
    if (alt && alt->Init(req.pb.planner_id())) {
      pipeline_->SetPlanner(alt);
    }
  } else {
    pipeline_->SetPlanner(planner_);
  }

  if (req.pb.has_goal_pose() && kinematics_ &&
      req.pb.planner_id() != "pilz_lin" &&
      req.pb.planner_id() != "pilz_circ" &&
      req.pb.planner_id() != "pilz_sequence") {
    automsgs::msgs::sensor_msgs::JointState seed = req.pb.start_state();
    automsgs::msgs::sensor_msgs::JointState ik;
    const auto code = kinematics_->GetPositionIK(req.pb.goal_pose(), seed, {}, &ik);
    metrics::ManipulationMetrics::Instance().RecordInverseKinematicsAttempt(code == ErrorCode::SUCCESS);
    if (code != ErrorCode::SUCCESS) {
      ::autonomy::manipulation::proto::MotionPlanResponse failed;
      failed.error = "IK failed";
      failed.error_code = code;
      metrics::ManipulationMetrics::Instance().RecordPlanAttemptEnd(false);
      return failed;
    }
    *req.pb.mutable_goal_state() = ik;
    req.pb.set_has_goal_pose(false);
  }

  if (!pipeline_) {
    ::autonomy::manipulation::proto::MotionPlanResponse failed;
    failed.error = "no pipeline";
    failed.error_code = ErrorCode::FAILURE;
    metrics::ManipulationMetrics::Instance().RecordPlanAttemptEnd(false);
    return failed;
  }
  auto response = pipeline_->Plan(req);
  if (response.success() && scene_ && response.trajectory().points_size() > 0) {
    scene_->SetCurrentState(MakeJointStateFromPoint(
        response.trajectory(), response.trajectory().points_size() - 1));
  }
  metrics::ManipulationMetrics::Instance().RecordPlanAttemptEnd(response.success());
  return response;
}

ErrorCode ManipulationServer::ExecuteTrajectory(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory, bool replace) {
  if (!execution_) {
    return ErrorCode::CONTROL_FAILED;
  }
  automsgs::msgs::trajectory_msgs::JointTrajectory traj = trajectory;
  // Optional inverse-dynamics feedforward into waypoint efforts.
  if (dynamics_ && traj.points_size() > 0) {
    for (int i = 0; i < traj.points_size(); ++i) {
      auto* pt = traj.mutable_points(i);
      if (pt->effort_size() > 0 || pt->positions_size() == 0) {
        continue;
      }
      std::vector<double> positions(pt->positions().begin(),
                                    pt->positions().end());
      std::vector<double> velocities(pt->velocities().begin(),
                                     pt->velocities().end());
      std::vector<double> tau;
      const bool have_vel =
          pt->velocities_size() == pt->positions_size() &&
          pt->velocities_size() > 0;
      const bool ok =
          have_vel ? dynamics_->GetCoriolisTorques(positions, velocities, &tau)
                   : dynamics_->GetGravityTorques(positions, &tau);
      if (ok && tau.size() == static_cast<std::size_t>(pt->positions_size())) {
        pt->clear_effort();
        for (double e : tau) {
          pt->add_effort(e);
        }
      }
    }
  }
  return execution_->Execute(traj, replace);
}

bool ManipulationServer::Execute(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory,
                                 bool replace) {
  return ExecuteTrajectory(trajectory, replace) == ErrorCode::SUCCESS;
}

void ManipulationServer::CancelExecution() {
  if (execution_) {
    execution_->Cancel();
  }
}

planner::PlanningPipeline* ManipulationServer::pipeline() {
  return pipeline_.get();
}

scene::PlanningScene* ManipulationServer::scene() {
  return scene_.get();
}

model::SimpleRobotModel* ManipulationServer::model() {
  return model_.get();
}

common::KinematicsInterface* ManipulationServer::kinematics() {
  return kinematics_.get();
}

dynamics::DynamicsSolver* ManipulationServer::dynamics_solver() {
  return dynamics_.get();
}

scene::SceneMonitor* ManipulationServer::scene_monitor() {
  return scene_monitor_.get();
}

execution::TrajectoryExecutionManager*
ManipulationServer::execution_manager() {
  return execution_.get();
}

dispatch::Capability* ManipulationServer::GetCapability(
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
    } else if (kv.first == "planning_time_milliseconds") {
      options_.set_planning_time_milliseconds(
          static_cast<uint32_t>(std::stoul(kv.second)));
      changed = true;
    } else if (kv.first == "max_velocity") {
      options_.set_max_velocity(std::stod(kv.second));
      changed = true;
    } else if (kv.first == "max_acceleration") {
      options_.set_max_acceleration(std::stod(kv.second));
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
