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
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/navigator/bt_navigator.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "autonomy/sensor/collator_interface.hpp"
#include "autonomy/sensor/consumer.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {

struct TaskAttachOptions
{
  bool enable_bt_tasks{true};
  std::string config_directory;
  std::string global_frame;
  std::string controller_id;
  std::string goal_checker_id;
};

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
  control::ControllerServer * controller_server() { return controller_server_.get(); }
  common::TaskContext * task_context() { return task_context_.get(); }
  navigator::NavigatorRegistry * navigatorRegistry() {
    return navigator_registry_.get();
  }

  void attachScheduler(const TaskAttachOptions & options);
  bool isSchedulerReady() const;

  behavior_tree::BtStatus navigateToPose(
    std::shared_ptr<const proto::NavigateToPoseAction::Goal> goal);
  behavior_tree::BtStatus navigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
    const std::string & behavior_tree = "");
  behavior_tree::BtStatus navigateToDock(
    const commsgs::geometry_msgs::PoseStamped & dock_pose,
    const std::string & dock_type = "default",
    const std::string & dock_id = "");
  behavior_tree::BtStatus trackToTarget(uint32_t target_id);
  behavior_tree::BtStatus exploreAnywhere(double time_allowance_sec = 0.0);
  behavior_tree::BtStatus teleopDrive(
    double time_allowance_sec = 0.0,
    double max_linear_vel = 0.0,
    double max_angular_vel = 0.0,
    std::function<bool()> cancel_checker = nullptr);

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
  sensor::CollatorInterface & sensorCollator();
  const sensor::CollatorInterface & sensorCollator() const;

  bool planToGoal(const commsgs::geometry_msgs::PoseStamped & goal);
  void replanToGoal(const commsgs::geometry_msgs::PoseStamped & goal);
  bool navigateToPose(
    const commsgs::geometry_msgs::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    std::function<bool()> continue_predicate = nullptr,
    double timeout_sec = 0.0);

  void setControllerEnabled(bool enabled);
  void requestCancelNavigation();
  commsgs::geometry_msgs::TwistStamped tickControl();
  std::optional<commsgs::planning_msgs::Path> lastPath() const;
  bool transformPoseToGlobalFrame(commsgs::geometry_msgs::PoseStamped & pose);
  bool useBehaviorTreeNavigation() const;

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

  behavior_tree::BtStatus runBlockingBt(
    std::function<behavior_tree::BtStatus()> task_fn,
    std::function<bool()> cancel_checker = nullptr,
    std::function<bool()> continue_predicate = nullptr);

  std::function<bool()> navigationCancelChecker();
  bool lastFollowPathSucceeded() const;
  bool lastFollowPathFailed() const;
  void notifyPlan(const commsgs::planning_msgs::Path & path);
  void setupSensorCollator();
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
  navigator::NavigatorRegistry::UniquePtr navigator_registry_;

  NavigationState nav_;
  std::vector<MapPublishListener> map_listeners_;
  std::vector<PathListener> path_listeners_;
  mutable std::mutex path_mutex_;
  std::optional<commsgs::planning_msgs::Path> last_path_;

  sensor::SensorConsumer sensor_consumer_;
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
};

}  // namespace tasks
}  // namespace autonomy
