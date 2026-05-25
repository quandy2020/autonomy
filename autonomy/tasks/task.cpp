/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/task.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <thread>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/sensor/internal/sensor_collator.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace {

void BeginTeleopSession(
  navigator::teleop::TeleopSession * session,
  const proto::TaskOptions & task_options,
  double max_linear_vel,
  double max_angular_vel)
{
  if (!session) {
    return;
  }
  navigator::teleop::TeleopSession::Limits limits;
  if (task_options.has_teleop_drive_options()) {
    const auto & opts = task_options.teleop_drive_options();
    limits.max_linear_vel = opts.default_max_linear_vel();
    limits.max_angular_vel = opts.default_max_angular_vel();
    limits.cmd_stale_timeout_sec = opts.cmd_stale_timeout_sec();
  }
  if (max_linear_vel > 0.0) {
    limits.max_linear_vel = max_linear_vel;
  }
  if (max_angular_vel > 0.0) {
    limits.max_angular_vel = max_angular_vel;
  }
  session->Begin(0.0, limits);
}

void ApplyMapToCostmap(
  const map::costmap_2d::Costmap2DWrapper::SharedPtr & wrapper,
  const commsgs::map_msgs::OccupancyGrid::SharedPtr & map)
{
  if (!wrapper || !map) {
    return;
  }
  if (!wrapper->applyOccupancyGrid(*map)) {
    AWARN << "Failed to apply map from MapServer to costmap wrapper";
  }
}

commsgs::geometry_msgs::PoseStamped poseFromOdometry(
  const commsgs::planning_msgs::Odometry & odom, const std::string & default_frame)
{
  commsgs::geometry_msgs::PoseStamped pose;
  pose.header = odom.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = default_frame;
  }
  pose.pose = odom.pose.pose;
  return pose;
}

TaskAttachOptions toAttachOptions(const RuntimeOptions & runtime)
{
  TaskAttachOptions opts;
  opts.enable_bt_tasks = runtime.enable_bt_tasks;
  opts.config_directory = runtime.config_directory;
  opts.global_frame = runtime.global_frame;
  opts.controller_id = runtime.controller_id;
  opts.goal_checker_id = runtime.goal_checker_id;
  return opts;
}

}  // namespace

Task::Task(const system::proto::AutonomyOptions & options) : options_{options}
{
  tf_buffer_ = transform::Buffer::Instance();
  map_server_ = std::make_shared<map::MapServer>(options_.map_options());
  controller_server_ = std::make_shared<control::ControllerServer>(
    options_.controller_options());
  planner_server_ =
    std::make_shared<planning::PlannerServer>(options_.planner_options());
  smoother_server_ = std::make_shared<planning::SmootherServer>(
    options_.planner_options(), planner_server_->GetCostmapWrapper());
  planner_server_->SetSmootherServer(smoother_server_);
  navigator_registry_ = std::make_unique<navigator::NavigatorRegistry>();
}

Task::~Task()
{
  shutdown();
}

void Task::start()
{
  if (started_) {
    return;
  }

  task_context_ = std::make_shared<common::TaskContext>();
  task_context_->planner = planner_server_;
  task_context_->smoother = smoother_server_;
  task_context_->controller = controller_server_;
  task_context_->tf_buffer = std::shared_ptr<transform::Buffer>(
    tf_buffer_, [](transform::Buffer *) {});
  if (controller_server_) {
    task_context_->odom_smoother = controller_server_->GetOdomSmoother();
  }
  setupSensorCollator();

  if (map_server_ != nullptr) {
    map_server_->SetMapPublishCallback(
      [this](const commsgs::map_msgs::OccupancyGrid::SharedPtr & map) {
        if (planner_server_) {
          ApplyMapToCostmap(planner_server_->GetCostmapWrapper(), map);
        }
        if (controller_server_) {
          ApplyMapToCostmap(controller_server_->GetCostmapWrapper(), map);
        }
        for (const auto & listener : map_listeners_) {
          if (listener) {
            listener(map);
          }
        }
      });
    map_server_->Start();
  }

  if (planner_server_ != nullptr) {
    planner_server_->Start();
  }
  if (smoother_server_ != nullptr) {
    smoother_server_->Start();
  }
  if (controller_server_ != nullptr) {
    const std::string global_frame =
      options_.planner_options().costmap().frame_id().empty()
        ? "map"
        : options_.planner_options().costmap().frame_id();
    std::shared_ptr<transform::Buffer> tf_shared(
      tf_buffer_, [](transform::Buffer *) {});
    controller_server_->SetNavigationContext(tf_shared, global_frame, "base_link");
    if (planner_server_) {
      controller_server_->SetSharedCostmap(planner_server_->GetCostmapWrapper());
    }
    controller_server_->Start();
  }

  if (map_server_ != nullptr) {
    const auto map = map_server_->GetStaticMapShared();
    if (map) {
      if (planner_server_) {
        ApplyMapToCostmap(planner_server_->GetCostmapWrapper(), map);
      } else if (controller_server_) {
        ApplyMapToCostmap(controller_server_->GetCostmapWrapper(), map);
      }
    }
  }

  started_ = true;
}

void Task::shutdown()
{
  if (navigator_registry_) {
    navigator_registry_->shutdown();
  }
  if (controller_server_ != nullptr) {
    controller_server_->Shutdown();
  }
  if (smoother_server_ != nullptr) {
    smoother_server_->Shutdown();
  }
  if (planner_server_ != nullptr) {
    planner_server_->Shutdown();
  }
  if (map_server_ != nullptr) {
    map_server_->Shutdown();
  }
  task_context_.reset();
  started_ = false;
}

void Task::attachScheduler(const TaskAttachOptions & options)
{
  if (!navigator_registry_) {
    navigator_registry_ = std::make_unique<navigator::NavigatorRegistry>();
  }
  navigator_registry_->shutdown();

  if (!options.enable_bt_tasks || !started_) {
    return;
  }
  if (!planner_server_ || !controller_server_ || !task_context_) {
    return;
  }

  navigator_registry_->attach(
    options.config_directory, task_context_, controller_server_);
}

bool Task::isSchedulerReady() const
{
  return navigator_registry_ && navigator_registry_->isReady();
}

behavior_tree::BtStatus Task::navigateToPose(
  std::shared_ptr<const proto::NavigateToPoseAction::Goal> goal)
{
  return navigator_registry_ ? navigator_registry_->navigateToPose(goal)
                      : behavior_tree::BtStatus::FAILED;
}

behavior_tree::BtStatus Task::runBlockingBt(
  std::function<behavior_tree::BtStatus()> task_fn,
  std::function<bool()> cancel_checker,
  std::function<bool()> continue_predicate)
{
  if (!isSchedulerReady() || !task_fn) {
    return behavior_tree::BtStatus::FAILED;
  }
  if (!continue_predicate) {
    continue_predicate = []() { return true; };
  }
  nav_.cancel.store(false);
  requestCancel();
  nav_.bt_active.store(true);
  setControllerEnabled(true);

  std::packaged_task<behavior_tree::BtStatus()> packaged(std::move(task_fn));
  auto future = packaged.get_future();
  std::thread worker(std::move(packaged));

  while (future.wait_for(std::chrono::milliseconds(kBtWaitPollMs)) !=
    std::future_status::ready)
  {
    if ((cancel_checker && cancel_checker()) || !continue_predicate()) {
      requestCancel();
    }
  }
  const auto status = future.get();
  worker.join();
  nav_.bt_active.store(false);
  return status;
}

behavior_tree::BtStatus Task::navigateThroughPoses(
  const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
  const std::string & behavior_tree)
{
  return runBlockingBt([this, poses, behavior_tree]() {
    return navigator_registry_->navigateThroughPoses(poses, behavior_tree);
  });
}

behavior_tree::BtStatus Task::navigateToDock(
  const commsgs::geometry_msgs::PoseStamped & dock_pose,
  const std::string & dock_type, const std::string & dock_id)
{
  return runBlockingBt([this, dock_pose, dock_type, dock_id]() {
    return navigator_registry_->navigateToDock(dock_pose, dock_type, dock_id);
  });
}

behavior_tree::BtStatus Task::trackToTarget(const uint32_t target_id)
{
  return runBlockingBt([this, target_id]() {
    return navigator_registry_->trackToTarget(target_id);
  });
}

behavior_tree::BtStatus Task::exploreAnywhere(const double time_allowance_sec)
{
  return runBlockingBt([this, time_allowance_sec]() {
    return navigator_registry_->exploreAnywhere(time_allowance_sec);
  });
}

behavior_tree::BtStatus Task::teleopDrive(
  const double time_allowance_sec,
  const double max_linear_vel,
  const double max_angular_vel,
  std::function<bool()> cancel_checker)
{
  if (!isSchedulerReady()) {
    return behavior_tree::BtStatus::FAILED;
  }
  nav_.teleop_active.store(true);
  setControllerEnabled(false);
  requestCancelNavigation();

  const auto status = navigator_registry_->teleopDrive(
    time_allowance_sec, max_linear_vel, max_angular_vel, std::move(cancel_checker));

  nav_.teleop_active.store(false);
  endTeleop();
  return status;
}

void Task::beginTeleop(const double max_linear_vel, const double max_angular_vel)
{
  if (!navigator_registry_) {
    return;
  }
  nav_.teleop_active.store(true);
  setControllerEnabled(false);
  requestCancelNavigation();
  BeginTeleopSession(
    navigator_registry_->teleopSession(), navigator_registry_->taskOptions(),
    max_linear_vel, max_angular_vel);
}

void Task::endTeleop()
{
  nav_.teleop_active.store(false);
  if (auto * session = navigator_registry_ ? navigator_registry_->teleopSession()
                                          : nullptr) {
    session->End();
  }
}

bool Task::isTeleopActive() const
{
  if (nav_.teleop_active.load()) {
    return true;
  }
  const auto * session =
    navigator_registry_ ? navigator_registry_->teleopSession() : nullptr;
  return session && session->IsActive();
}

bool Task::updateTeleopCommand(const commsgs::geometry_msgs::TwistStamped & cmd)
{
  auto * session =
    navigator_registry_ ? navigator_registry_->teleopSession() : nullptr;
  if (!session) {
    return false;
  }
  session->UpdateCommand(cmd);
  return true;
}

bool Task::updateTrackTargetPose(const commsgs::geometry_msgs::PoseStamped & pose)
{
  return navigator_registry_ && navigator_registry_->updateTrackTargetPose(pose);
}

bool Task::updateExploreGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  return navigator_registry_ && navigator_registry_->updateExploreGoal(goal);
}

bool Task::hasNavigator(const std::string & id) const
{
  return navigator_registry_ && navigator_registry_->hasNavigator(id);
}

std::vector<std::string> Task::registeredNavigatorIds() const
{
  return navigator_registry_ ? navigator_registry_->registeredNavigatorIds()
                       : std::vector<std::string>{};
}

void Task::requestCancel()
{
  if (navigator_registry_) {
    navigator_registry_->requestCancel();
  }
}

void Task::configure(const RuntimeOptions & options)
{
  runtime_ = options;
  if (runtime_.planner_id.empty() && planner_server_) {
    runtime_.planner_id = planner_server_->GetDefaultPlannerId();
  }
  attachScheduler(toAttachOptions(runtime_));
  configured_ = true;
}

void Task::addMapPublishListener(MapPublishListener listener)
{
  if (listener) {
    map_listeners_.push_back(std::move(listener));
  }
}

void Task::addPathListener(PathListener listener)
{
  if (listener) {
    path_listeners_.push_back(std::move(listener));
  }
}

sensor::CollatorInterface & Task::sensorCollator()
{
  if (!sensor_collator_) {
    setupSensorCollator();
  }
  return *sensor_collator_;
}

const sensor::CollatorInterface & Task::sensorCollator() const
{
  return const_cast<Task *>(this)->sensorCollator();
}

void Task::setupSensorCollator()
{
  if (sensor_collator_) {
    return;
  }

  sensor_consumer_.controller = controller_server_.get();
  if (planner_server_) {
    if (auto wrapper = planner_server_->GetCostmapWrapper()) {
      auto * costmap = wrapper.get();
      sensor_consumer_.on_laser_scan =
        [costmap](const commsgs::sensor_msgs::LaserScan & scan) {
          costmap->feedLaserScan(scan);
        };
      sensor_consumer_.on_point_cloud =
        [costmap](const commsgs::sensor_msgs::PointCloud2 & cloud) {
          costmap->feedPointCloud2(cloud);
        };
      sensor_consumer_.on_range =
        [costmap](const commsgs::sensor_msgs::Range & range) {
          costmap->feedRange(range);
        };
    }
  }

  auto collator = std::make_unique<sensor::SensorCollator>(sensor_consumer_);
  collator->SetDispatchCallback(
    [this](const std::string & /*sensor_id*/, std::unique_ptr<sensor::Data> data) {
      if (data) {
        data->Dispatch(sensor_consumer_);
      }
    });
  sensor_collator_ = std::move(collator);
}

bool Task::useBehaviorTreeNavigation() const
{
  return configured_ && runtime_.use_bt_navigation && isSchedulerReady();
}

std::function<bool()> Task::navigationCancelChecker()
{
  return [this]() { return nav_.cancel.load(); };
}

bool Task::planToGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  nav_.cancel.store(false);
  if (!planner_server_ || !controller_server_ || !controller_server_->HasOdometry()) {
    AWARN << "[task] cannot plan: planner or odom missing";
    return false;
  }

  commsgs::planning_msgs::Odometry odom;
  if (!controller_server_->GetLatestOdometry(odom)) {
    AWARN << "[task] cannot plan: no odometry";
    return false;
  }

  auto start = poseFromOdometry(odom, runtime_.global_frame);
  auto goal_com = goal;
  if (goal_com.header.frame_id.empty()) {
    goal_com.header.frame_id = runtime_.global_frame;
  }

  try {
    const auto path = planner_server_->ComputePathToPose(
      start, goal_com, runtime_.planner_id, navigationCancelChecker());
    if (path.poses.size() < kMinPathPoses) {
      AWARN << "[task] planner returned empty path";
      return false;
    }

    notifyPlan(path);

    if (!controller_server_->BeginFollowPath(
        path, runtime_.controller_id, runtime_.goal_checker_id,
        runtime_.progress_checker_id))
    {
      AWARN << "[task] BeginFollowPath failed";
      return false;
    }
    nav_.following_path.store(true);
    AINFO << "[task] plan ready (" << path.poses.size() << " poses)";
    return true;
  } catch (const planning::common::PlannerException & e) {
    AWARN << "[task] planning failed: " << e.what();
  } catch (const std::exception & e) {
    AWARN << "[task] planning error: " << e.what();
  }
  return false;
}

void Task::replanToGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  if (!started_) {
    AWARN << "[task] replanToGoal ignored: not started";
    return;
  }
  if (!planToGoal(goal)) {
    return;
  }
  setControllerEnabled(true);
}

void Task::setControllerEnabled(bool enabled)
{
  nav_.controller_enabled.store(enabled);
  if (!enabled) {
    requestCancelNavigation();
    if (controller_server_ && nav_.following_path.load()) {
      controller_server_->EndFollowPath();
      nav_.following_path.store(false);
    }
  }
}

bool Task::navigateToPose(
  const commsgs::geometry_msgs::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  std::function<bool()> continue_predicate,
  const double timeout_sec)
{
  if (!continue_predicate) {
    continue_predicate = []() { return true; };
  }

  if (useBehaviorTreeNavigation()) {
    auto goal_com = goal;
    if (goal_com.header.frame_id.empty()) {
      goal_com.header.frame_id = runtime_.global_frame;
    }
    auto proto_goal = std::make_shared<
      proto::NavigateToPoseAction::Goal>();
    *proto_goal->mutable_pose() = commsgs::geometry_msgs::ToProto(goal_com);

    const auto status = runBlockingBt(
      [this, proto_goal]() { return navigateToPose(proto_goal); },
      cancel_checker, continue_predicate);

    if (cancel_checker && cancel_checker()) {
      return false;
    }
    return status == behavior_tree::BtStatus::SUCCEEDED;
  }

  replanToGoal(goal);
  return waitForDirectNavigation(
    goal, runtime_.goal_tolerance, cancel_checker, continue_predicate, timeout_sec);
}

void Task::requestCancelNavigation()
{
  nav_.cancel.store(true);
  requestCancel();
}

bool Task::lastFollowPathSucceeded() const
{
  using Tick = control::ControllerServer::FollowPathTickResult;
  return static_cast<Tick>(nav_.last_follow_result.load()) == Tick::Succeeded;
}

bool Task::lastFollowPathFailed() const
{
  using Tick = control::ControllerServer::FollowPathTickResult;
  return static_cast<Tick>(nav_.last_follow_result.load()) == Tick::Failed;
}

void Task::notifyPlan(const commsgs::planning_msgs::Path & path)
{
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    last_path_ = path;
  }
  for (const auto & listener : path_listeners_) {
    if (listener) {
      listener(path);
    }
  }
}

std::optional<commsgs::planning_msgs::Path> Task::lastPath() const
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  return last_path_;
}

bool Task::waitForDirectNavigation(
  const commsgs::geometry_msgs::PoseStamped & goal, double tolerance,
  std::function<bool()> cancel_checker,
  std::function<bool()> continue_predicate, const double timeout_sec)
{
  if (!nav_.following_path.load()) {
    return false;
  }
  const double limit_sec = timeout_sec > 0.0 ? timeout_sec : kDirectNavDefaultTimeoutSec;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(limit_sec);
  const auto sleep_duration = std::chrono::milliseconds(1000 / kSpinRateHz);

  while (continue_predicate() && nav_.following_path.load()) {
    if ((cancel_checker && cancel_checker()) || nav_.cancel.load()) {
      return false;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    if (lastFollowPathSucceeded()) {
      return true;
    }
    if (lastFollowPathFailed()) {
      return false;
    }
    if (controller_server_ && controller_server_->HasOdometry()) {
      commsgs::planning_msgs::Odometry odom;
      if (controller_server_->GetLatestOdometry(odom)) {
        const double dx = goal.pose.position.x - odom.pose.pose.position.x;
        const double dy = goal.pose.position.y - odom.pose.pose.position.y;
        if (std::hypot(dx, dy) < tolerance) {
          return true;
        }
      }
    }
    std::this_thread::sleep_for(sleep_duration);
  }
  return lastFollowPathSucceeded();
}

commsgs::geometry_msgs::TwistStamped Task::tickControl()
{
  commsgs::geometry_msgs::TwistStamped zero;
  if (!started_) {
    return zero;
  }

  if (nav_.teleop_active.load() && navigator_registry_) {
    commsgs::geometry_msgs::TwistStamped zero;
    auto * session = navigator_registry_->teleopSession();
    if (!session || !session->IsActive()) {
      return zero;
    }
    session->Tick([]() { return false; });
    return session->CurrentCommand();
  }

  if (!nav_.controller_enabled.load() || !controller_server_) {
    return zero;
  }

  if (nav_.bt_active.load()) {
    return controller_server_->GetLastCmdVel();
  }

  if (!nav_.following_path.load()) {
    return zero;
  }

  using Tick = control::ControllerServer::FollowPathTickResult;
  const auto tick = controller_server_->TickFollowPath(navigationCancelChecker());
  nav_.last_follow_result.store(static_cast<int>(tick));

  if (tick != Tick::Running) {
    controller_server_->EndFollowPath();
    nav_.following_path.store(false);
    if (tick == Tick::Succeeded) {
      AINFO << "[task] controller reached goal";
    } else if (tick == Tick::Failed) {
      AWARN << "[task] controller follow failed";
    }
  }

  return controller_server_->GetLastCmdVel();
}

bool Task::transformPoseToGlobalFrame(commsgs::geometry_msgs::PoseStamped & pose)
{
  if (!planner_server_) {
    return false;
  }
  auto wrapper = planner_server_->GetCostmapWrapper();
  if (!wrapper) {
    return false;
  }
  commsgs::geometry_msgs::PoseStamped transformed;
  if (!wrapper->transformPoseToGlobalFrame(pose, transformed)) {
    return false;
  }
  pose = transformed;
  return true;
}

}  // namespace tasks
}  // namespace autonomy
