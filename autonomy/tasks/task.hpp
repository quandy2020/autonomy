/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/scheduler/task_scheduler.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {

/** @brief Parameters for attaching BT / TaskScheduler after @ref Task::start(). */
struct TaskAttachOptions
{
  bool enable_bt_tasks{true};
  std::string config_directory;
  std::string global_frame;
  std::string controller_id;
  std::string goal_checker_id;
};

/** @brief Runtime navigation / controller IDs (from launch params or host config). */
struct RuntimeOptions
{
  bool enable_bt_tasks{true};
  bool use_bt_navigation{true};
  std::string config_directory;
  std::string planner_id;
  std::string controller_id;
  std::string goal_checker_id;
  std::string progress_checker_id;
  std::string global_frame;
  double goal_tolerance{0.25};
};

/**
 * @brief Core runtime: map / planner / controller / smoother / task context / BT scheduler.
 */
class Task
{
public:
  AUTONOMY_SMART_PTR_DEFINITIONS(Task)

  using MapPublishListener = std::function<void(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr & map)>;
  using PathListener = std::function<void(const commsgs::planning_msgs::Path & path)>;

  Task() = default;

  explicit Task(const system::proto::AutonomyOptions & options);

  ~Task();

  void start();

  void shutdown();

  bool isStarted() const { return started_; }

  void configure(const RuntimeOptions & options);

  transform::Buffer * tf_buffer() { return tf_buffer_; }

  map::MapServer * map_server() { return map_server_.get(); }

  planning::PlannerServer * planner_server() { return planner_server_.get(); }

  planning::SmootherServer * smoother_server() { return smoother_server_.get(); }

  control::ControllerServer * controller_server() {
    return controller_server_.get();
  }

  common::TaskContext * task_context() { return task_context_.get(); }

  void attachScheduler(const TaskAttachOptions & options);

  bool isSchedulerReady() const;

  scheduler::TaskScheduler * scheduler() { return scheduler_.get(); }

  behavior_tree::BtStatus navigateToPose(
    std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal> goal);

  behavior_tree::BtStatus navigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
    const std::string & behavior_tree = "");

  behavior_tree::BtStatus navigateToDock(
    const commsgs::geometry_msgs::PoseStamped & dock_pose,
    const std::string & dock_type = "default",
    const std::string & dock_id = "");

  behavior_tree::BtStatus trackToTarget(uint32_t target_id);

  behavior_tree::BtStatus exploreAnywhere(double time_allowance_sec = 0.0);

  /** @brief 阻塞遥操会话（带超时）。 */
  behavior_tree::BtStatus teleopDrive(
    double time_allowance_sec = 0.0,
    double max_linear_vel = 0.0,
    double max_angular_vel = 0.0,
    std::function<bool()> cancel_checker = nullptr);

  /** @brief 非阻塞开启/关闭遥操（SetTeleopMode 用）。 */
  void beginTeleop(double max_linear_vel = 0.0, double max_angular_vel = 0.0);
  void endTeleop();
  bool isTeleopActive() const;

  bool updateTeleopCommand(const commsgs::geometry_msgs::TwistStamped & cmd);

  bool updateTrackTargetPose(const commsgs::geometry_msgs::PoseStamped & pose);
  bool updateExploreGoal(const commsgs::geometry_msgs::PoseStamped & goal);

  bool hasNavigator(const std::string & id) const;
  std::vector<std::string> registeredNavigatorIds() const;

  void requestCancel();

  void addMapPublishListener(MapPublishListener listener);
  void addPathListener(PathListener listener);

  void updateOdometry(const commsgs::planning_msgs::Odometry & odom);
  void feedLaserScan(const commsgs::sensor_msgs::LaserScan & scan);
  void feedPointCloud2(const commsgs::sensor_msgs::PointCloud2 & cloud);
  void feedRange(const commsgs::sensor_msgs::Range & range);

  bool planToGoal(const commsgs::geometry_msgs::PoseStamped & goal);
  void replanToGoal(const commsgs::geometry_msgs::PoseStamped & goal);

  bool navigateToPose(
    const commsgs::geometry_msgs::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    std::function<bool()> continue_predicate = nullptr,
    double timeout_sec = 0.0);

  void setControllerEnabled(bool enabled);
  void requestCancelNavigation();
  void applySpeedLimit(const commsgs::planning_msgs::SpeedLimit & limit);
  void clearSpeedLimit();

  commsgs::geometry_msgs::TwistStamped tickControl();

  std::optional<commsgs::planning_msgs::Path> lastPath() const;

  bool transformPoseToGlobalFrame(commsgs::geometry_msgs::PoseStamped & pose);

  bool useBehaviorTreeNavigation() const;

  behavior_tree::BtStatus runBlockingBtTask(
    std::function<behavior_tree::BtStatus()> task_fn,
    std::function<bool()> cancel_checker = nullptr,
    std::function<bool()> continue_predicate = nullptr);

private:
  struct NavigationState
  {
    std::atomic<bool> controller_enabled{false};
    std::atomic<bool> bt_active{false};
    std::atomic<bool> teleop_active{false};
    std::atomic<bool> following_path{false};
    std::atomic<bool> cancel{false};
    std::atomic<int> last_follow_result{0};
  };

  std::function<bool()> navigationCancelChecker();
  bool lastFollowPathSucceeded() const;
  bool lastFollowPathFailed() const;
  void notifyPlan(const commsgs::planning_msgs::Path & path);
  bool waitForDirectNavigation(
    const commsgs::geometry_msgs::PoseStamped & goal, double tolerance,
    std::function<bool()> cancel_checker,
    std::function<bool()> continue_predicate, double timeout_sec);

  system::proto::AutonomyOptions options_;
  RuntimeOptions runtime_;
  bool configured_{false};
  bool started_{false};

  transform::Buffer * tf_buffer_{nullptr};

  map::MapServer::SharedPtr map_server_{nullptr};

  control::ControllerServer::SharedPtr controller_server_{nullptr};

  planning::PlannerServer::SharedPtr planner_server_{nullptr};

  planning::SmootherServer::SharedPtr smoother_server_{nullptr};

  std::shared_ptr<common::TaskContext> task_context_{nullptr};

  std::unique_ptr<scheduler::TaskScheduler> scheduler_;

  NavigationState nav_;
  std::vector<MapPublishListener> map_listeners_;
  std::vector<PathListener> path_listeners_;
  mutable std::mutex path_mutex_;
  std::optional<commsgs::planning_msgs::Path> last_path_;
};

}  // namespace tasks
}  // namespace autonomy
