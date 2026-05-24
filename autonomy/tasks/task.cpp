/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/task.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"

namespace autonomy {
namespace tasks {
namespace {

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
  task_context_->global_costmap = planner_server_
    ? planner_server_->GetCostmapWrapper()
    : nullptr;
  task_context_->local_costmap = controller_server_
    ? controller_server_->GetCostmapWrapper()
    : task_context_->global_costmap;
  task_context_->tf = std::shared_ptr<transform::Buffer>(
    tf_buffer_, [](transform::Buffer *) {});
  if (controller_server_) {
    task_context_->odom_smoother = controller_server_->GetOdomSmoother();
  }
  task_context_->teleop_session =
    std::make_shared<navigator::teleop::TeleopSession>();
  if (planner_server_) {
    const auto & planner_options = options_.planner_options();
    if (!planner_options.default_planner_id().empty()) {
      task_context_->selected_planner_id = planner_options.default_planner_id();
    } else {
      task_context_->selected_planner_id = planner_server_->GetDefaultPlannerId();
    }
    if (!planner_options.default_smoother_id().empty()) {
      task_context_->selected_smoother_id = planner_options.default_smoother_id();
    } else if (smoother_server_) {
      task_context_->selected_smoother_id = smoother_server_->GetDefaultSmootherId();
    }
    task_context_->global_frame =
      planner_options.costmap().frame_id().empty()
        ? "map"
        : planner_options.costmap().frame_id();
  }

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
  if (scheduler_) {
    scheduler_->Shutdown();
    scheduler_.reset();
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
  if (scheduler_) {
    scheduler_->Shutdown();
    scheduler_.reset();
  }

  if (!options.enable_bt_tasks || !started_) {
    return;
  }

  if (!planner_server_ || !controller_server_ || !task_context_) {
    return;
  }

  scheduler_ = std::make_unique<scheduler::TaskScheduler>();
  scheduler::SharedSystem attachment;
  attachment.planner = planner_server_;
  attachment.smoother = smoother_server_;
  attachment.controller = controller_server_;
  attachment.task_context = task_context_;
  scheduler_->InitializeAttached(options.config_directory, attachment);

  if (!scheduler_->IsInitialized()) {
    scheduler_.reset();
    return;
  }

  if (auto * ctx = task_context_.get()) {
    if (!options.global_frame.empty()) {
      ctx->global_frame = options.global_frame;
    }
    if (!options.controller_id.empty()) {
      ctx->selected_controller_id = options.controller_id;
    }
    if (!options.goal_checker_id.empty()) {
      ctx->selected_goal_checker_id = options.goal_checker_id;
    }
  }
}

bool Task::isSchedulerReady() const
{
  return scheduler_ != nullptr && scheduler_->IsInitialized();
}

behavior_tree::BtStatus Task::navigateToPose(
  std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal> goal)
{
  if (!isSchedulerReady() || !goal) {
    return behavior_tree::BtStatus::FAILED;
  }
  return scheduler_->NavigateToPose(goal);
}

behavior_tree::BtStatus Task::runBlockingBtTask(
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
  return runBlockingBtTask(
    [this, poses, behavior_tree]() {
      return scheduler_->NavigateThroughPoses(poses, behavior_tree);
    });
}

behavior_tree::BtStatus Task::navigateToDock(
  const commsgs::geometry_msgs::PoseStamped & dock_pose,
  const std::string & dock_type, const std::string & dock_id)
{
  return runBlockingBtTask([this, dock_pose, dock_type, dock_id]() {
    return scheduler_->NavigateToDockPose(dock_pose, dock_type, dock_id);
  });
}

behavior_tree::BtStatus Task::trackToTarget(const uint32_t target_id)
{
  return runBlockingBtTask([this, target_id]() {
    return scheduler_->TrackToTarget(target_id);
  });
}

behavior_tree::BtStatus Task::exploreAnywhere(const double time_allowance_sec)
{
  return runBlockingBtTask([this, time_allowance_sec]() {
    return scheduler_->ExploreToAnywhere(time_allowance_sec);
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

  const auto status = scheduler_->TeleopDrive(
    time_allowance_sec, max_linear_vel, max_angular_vel, std::move(cancel_checker));

  nav_.teleop_active.store(false);
  if (scheduler_) {
    scheduler_->EndTeleopSession();
  }
  return status;
}

void Task::beginTeleop(const double max_linear_vel, const double max_angular_vel)
{
  if (!scheduler_) {
    return;
  }
  nav_.teleop_active.store(true);
  setControllerEnabled(false);
  requestCancelNavigation();
  scheduler_->BeginTeleopSession(max_linear_vel, max_angular_vel);
}

void Task::endTeleop()
{
  nav_.teleop_active.store(false);
  if (scheduler_) {
    scheduler_->EndTeleopSession();
  }
}

bool Task::isTeleopActive() const
{
  return nav_.teleop_active.load() ||
    (scheduler_ && scheduler_->IsTeleopActive());
}

bool Task::updateTeleopCommand(const commsgs::geometry_msgs::TwistStamped & cmd)
{
  if (task_context_ && task_context_->teleop_session &&
    task_context_->teleop_session->IsActive())
  {
    task_context_->teleop_session->UpdateCommand(cmd);
    return true;
  }
  return scheduler_ && scheduler_->UpdateTeleopCommand(cmd);
}

bool Task::updateTrackTargetPose(const commsgs::geometry_msgs::PoseStamped & pose)
{
  return scheduler_ && scheduler_->UpdateTrackTargetPose(pose);
}

bool Task::updateExploreGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  return scheduler_ && scheduler_->UpdateExploreGoal(goal);
}

bool Task::hasNavigator(const std::string & id) const
{
  return scheduler_ && scheduler_->HasNavigator(id);
}

std::vector<std::string> Task::registeredNavigatorIds() const
{
  if (!scheduler_) {
    return {};
  }
  return scheduler_->RegisteredNavigatorIds();
}

void Task::requestCancel()
{
  if (scheduler_) {
    scheduler_->RequestCancel();
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

void Task::updateOdometry(const commsgs::planning_msgs::Odometry & odom)
{
  if (controller_server_) {
    controller_server_->UpdateOdometry(odom);
  }
}

void Task::feedLaserScan(const commsgs::sensor_msgs::LaserScan & scan)
{
  if (planner_server_) {
    if (auto wrapper = planner_server_->GetCostmapWrapper()) {
      wrapper->feedLaserScan(scan);
    }
  }
}

void Task::feedPointCloud2(const commsgs::sensor_msgs::PointCloud2 & cloud)
{
  if (planner_server_) {
    if (auto wrapper = planner_server_->GetCostmapWrapper()) {
      wrapper->feedPointCloud2(cloud);
    }
  }
}

void Task::feedRange(const commsgs::sensor_msgs::Range & range)
{
  if (planner_server_) {
    if (auto wrapper = planner_server_->GetCostmapWrapper()) {
      wrapper->feedRange(range);
    }
  }
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

void Task::applySpeedLimit(const commsgs::planning_msgs::SpeedLimit & limit)
{
  if (controller_server_) {
    controller_server_->ApplySpeedLimit(limit);
  }
}

void Task::clearSpeedLimit()
{
  commsgs::planning_msgs::SpeedLimit cleared;
  cleared.header.stamp = commsgs::builtin_interfaces::Time::Now();
  cleared.percentage = false;
  cleared.speed_limit = kClearedSpeedLimit;
  applySpeedLimit(cleared);
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
      behavior_tree::proto::NavigateToPoseAction::Goal>();
    *proto_goal->mutable_pose() = commsgs::geometry_msgs::ToProto(goal_com);

    const auto status = runBlockingBtTask(
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

  if (nav_.teleop_active.load() && task_context_ && task_context_->teleop_session &&
    task_context_->teleop_session->IsActive())
  {
    task_context_->teleop_session->Tick([]() { return false; });
    return task_context_->teleop_session->CurrentCommand();
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
