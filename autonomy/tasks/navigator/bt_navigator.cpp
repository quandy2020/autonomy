/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigator/bt_navigator.hpp"

#include <algorithm>
#include <atomic>
#include <unordered_map>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/tasks/common/bt_navigator.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/navigator/docking/navigate_to_docking.hpp"
#include "autonomy/tasks/navigator/exploration/explore_to_anywhere.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_through_poses.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_drive.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"
#include "autonomy/tasks/navigator/tracking/track_to_target.hpp"
#include "autonomy/tasks/options.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace {

using CancelFn = std::function<bool()>;

std::string DefaultBehaviorTreeFile(const proto::NavigatorConfig & config,
                                    const char * fallback)
{
  return config.default_behavior_tree_file().empty()
           ? fallback
           : config.default_behavior_tree_file();
}

common::FeedbackUtils WithDefaultBt(common::FeedbackUtils feedback,
                                    const proto::NavigatorConfig & config,
                                    const char * fallback)
{
  feedback.default_bt_xml_filename =
    DefaultBehaviorTreeFile(config, fallback);
  return feedback;
}

template <typename NavigatorT>
std::unique_ptr<common::NavigatorBase> MakeNavigator(
  const proto::TaskOptions & options,
  const std::shared_ptr<common::TaskContext> & task_context,
  const std::vector<std::string> & plugin_libs,
  common::FeedbackUtils feedback,
  const std::shared_ptr<common::NavigatorMuxer> & muxer,
  const std::shared_ptr<control::utils::OdomSmoother> & odom_smoother,
  const proto::NavigatorConfig & config,
  const char * default_bt)
{
  feedback = WithDefaultBt(std::move(feedback), config, default_bt);
  return std::make_unique<NavigatorT>(
    options, task_context, plugin_libs, feedback, muxer, odom_smoother);
}

std::unique_ptr<common::NavigatorBase> CreateBuiltinNavigator(
  const std::string & id,
  const proto::TaskOptions & options,
  const std::shared_ptr<common::TaskContext> & task_context,
  const std::vector<std::string> & plugin_libs,
  common::FeedbackUtils feedback,
  const std::shared_ptr<common::NavigatorMuxer> & muxer,
  const std::shared_ptr<control::utils::OdomSmoother> & odom_smoother,
  const proto::NavigatorConfig & config)
{
  if (id == kNavigatorNavigateToPose) {
    return MakeNavigator<navigation::NavigateToPoseNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "navigate_to_pose.xml");
  }
  if (id == kNavigatorNavigateThroughPoses) {
    return MakeNavigator<navigation::NavigateThroughPosesNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "navigate_through_poses.xml");
  }
  if (id == kNavigatorNavigateToDocking) {
    return MakeNavigator<docking::NavigateToDockingNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "navigate_to_dock.xml");
  }
  if (id == kNavigatorTrackToTarget) {
    return MakeNavigator<tracking::TrackToTargetNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "track_to_target.xml");
  }
  if (id == kNavigatorExploreToAnywhere) {
    return MakeNavigator<exploration::ExploreToAnywhereNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "explore_to_anywhere.xml");
  }
  if (id == kNavigatorTeleopDrive) {
    return MakeNavigator<teleop::TeleopDriveNavigator>(
      options, task_context, plugin_libs, feedback, muxer, odom_smoother, config,
      "teleop_drive.xml");
  }
  return nullptr;
}

std::string ResolveBehaviorTreePath(const std::string & configuration_directory,
                                    const std::string & filename)
{
  return configuration_directory + "/tasks/behavior_tree/" + filename;
}

const proto::NavigatorConfig & NavigatorConfigFor(
  const proto::TaskOptions & options, const std::string & id)
{
  if (id == kNavigatorNavigateToPose) {
    return options.navigate_to_pose();
  }
  if (id == kNavigatorNavigateThroughPoses) {
    return options.navigate_through_poses();
  }
  if (id == kNavigatorNavigateToDocking) {
    return options.navigate_to_docking();
  }
  if (id == kNavigatorTrackToTarget) {
    return options.track_to_target();
  }
  if (id == kNavigatorExploreToAnywhere) {
    return options.explore_to_anywhere();
  }
  if (id == kNavigatorTeleopDrive) {
    return options.teleop_drive();
  }
  static const proto::NavigatorConfig kEmpty{};
  return kEmpty;
}

}  // namespace

struct NavigatorRegistry::Impl
{
  using CancelFn = std::function<bool()>;

  void shutdown()
  {
    navigators.clear();
    ready = false;
    task_context.reset();
  }

  template <typename NavigatorT>
  std::shared_ptr<NavigatorT> get(const std::string & id) const
  {
    const auto it = navigators.find(id);
    if (it == navigators.end()) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<NavigatorT>(it->second);
  }

  const teleop::TeleopSession * teleopSession() const
  {
    auto nav = get<teleop::TeleopDriveNavigator>(kNavigatorTeleopDrive);
    return nav && nav->session() ? nav->session().get() : nullptr;
  }

  void applyTaskOptionsToContext()
  {
    if (!task_context || !task_context->controller) {
      return;
    }
    const std::string global_frame =
      !task_options_.global_frame().empty() ? task_options_.global_frame() : "map";
    const std::string robot_frame =
      !task_options_.robot_base_frame().empty()
        ? task_options_.robot_base_frame()
        : "base_link";
    task_context->controller->SetNavigationContext(
      task_context->tf_buffer, global_frame, robot_frame);
  }

  void setupNavigators()
  {
    std::vector<std::string> plugin_libs;
    plugin_libs.reserve(task_options_.plugin_lib_names_size());
    for (const auto & name : task_options_.plugin_lib_names()) {
      plugin_libs.push_back(name);
    }

    common::FeedbackUtils feedback;
    feedback.global_frame =
      !task_options_.global_frame().empty() ? task_options_.global_frame() : "map";
    feedback.robot_frame =
      !task_options_.robot_base_frame().empty()
        ? task_options_.robot_base_frame()
        : "base_link";
    feedback.tf = task_context->tf_buffer;
    feedback.local_survival_timeout =
      task_options_.local_survival_timeout() > 0.0
        ? task_options_.local_survival_timeout()
        : 120.0;
    const std::string config_dir = configuration_directory_;
    feedback.bt_xml_path_resolver =
      [config_dir](const std::string & filename) {
        return ResolveBehaviorTreePath(config_dir, filename);
      };

    auto muxer_alias = std::shared_ptr<common::NavigatorMuxer>(
      &navigator_muxer_, [](common::NavigatorMuxer *) {});

    for (const auto & id : task_options_.navigators()) {
      const auto & nav_cfg = NavigatorConfigFor(task_options_, id);
      if (!nav_cfg.enable()) {
        AINFO << "Navigator '" << id << "' disabled in config.";
        continue;
      }
      auto instance = CreateBuiltinNavigator(
        id, task_options_, task_context, plugin_libs, feedback, muxer_alias,
        task_context->odom_smoother, nav_cfg);
      if (!instance) {
        AWARN << "Unknown or unsupported navigator id '" << id << "'; skipping.";
        continue;
      }
      navigators[id] = std::move(instance);
      AINFO << "Registered navigator '" << id << "' (BT: "
            << (nav_cfg.default_behavior_tree_file().empty()
                  ? "default"
                  : nav_cfg.default_behavior_tree_file())
            << ").";
    }
  }

  behavior_tree::BtStatus runNavigator(
    const std::string & id,
    const std::function<behavior_tree::BtStatus(CancelFn)> & run)
  {
    if (!ready) {
      AERROR << "NavigatorRegistry not attached.";
      return behavior_tree::BtStatus::FAILED;
    }
    if (navigators.find(id) == navigators.end()) {
      AERROR << "Navigator '" << id << "' is disabled or not registered.";
      return behavior_tree::BtStatus::FAILED;
    }
    cancel_requested.store(false);
    CancelFn cancel_fn = [this]() { return cancel_requested.load(); };
    return run(cancel_fn);
  }

  std::string configuration_directory_;
  proto::TaskOptions task_options_;
  std::shared_ptr<common::TaskContext> task_context;
  std::shared_ptr<control::ControllerServer> controller;
  std::unordered_map<std::string, std::shared_ptr<common::NavigatorBase>>
    navigators;
  common::NavigatorMuxer navigator_muxer_;
  std::atomic<bool> cancel_requested{false};
  bool ready{false};
};

NavigatorRegistry::NavigatorRegistry()
: impl_(std::make_unique<Impl>())
{
}

NavigatorRegistry::~NavigatorRegistry() = default;

void NavigatorRegistry::shutdown()
{
  impl_->shutdown();
}

void NavigatorRegistry::attach(
  const std::string & config_directory,
  std::shared_ptr<common::TaskContext> task_context,
  std::shared_ptr<control::ControllerServer> controller)
{
  impl_->shutdown();

  if (!task_context || !controller) {
    AERROR << "NavigatorRegistry::attach requires task_context and controller.";
    return;
  }

  impl_->task_context = std::move(task_context);
  impl_->controller = std::move(controller);

  impl_->configuration_directory_ =
    ::autonomy::common::ResolveConfigurationRootDirectory(
      config_directory, "tasks/tasks.lua");
  if (!config_directory.empty() &&
      config_directory != impl_->configuration_directory_) {
    AWARN << "Configuration directory '" << config_directory
          << "' was not found; using '" << impl_->configuration_directory_
          << "'.";
  }
  impl_->task_options_ =
    CreateOptions(impl_->configuration_directory_, "tasks/tasks.lua");

  impl_->applyTaskOptionsToContext();

  if (impl_->task_context->planner) {
    impl_->controller->SetSharedCostmap(
      impl_->task_context->planner->GetCostmapWrapper());
  }

  impl_->setupNavigators();
  impl_->ready = true;
  AINFO << "NavigatorRegistry ready (" << impl_->navigators.size()
        << " navigator(s)).";
}

bool NavigatorRegistry::isReady() const
{
  return impl_->ready;
}

void NavigatorRegistry::requestCancel()
{
  impl_->cancel_requested.store(true);
  for (auto & entry : impl_->navigators) {
    if (entry.second) {
      entry.second->RequestCancel();
    }
  }
}

behavior_tree::BtStatus NavigatorRegistry::navigateToPose(
  std::shared_ptr<const proto::NavigateToPoseAction::Goal> goal)
{
  if (!goal) {
    return behavior_tree::BtStatus::FAILED;
  }
  return impl_->runNavigator(kNavigatorNavigateToPose,
    [&](Impl::CancelFn cancel) {
      auto nav = impl_->get<navigation::NavigateToPoseNavigator>(
        kNavigatorNavigateToPose);
      return nav->Run(goal, cancel);
    });
}

behavior_tree::BtStatus NavigatorRegistry::navigateThroughPoses(
  const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
  const std::string & behavior_tree)
{
  if (poses.empty()) {
    AERROR << "NavigateThroughPoses requires at least one pose.";
    return behavior_tree::BtStatus::FAILED;
  }
  auto goal = std::make_shared<
    proto::NavigateThroughPosesAction::Goal>();
  for (const auto & pose : poses) {
    *goal->mutable_poses()->add_goals() = commsgs::geometry_msgs::ToProto(pose);
  }
  if (!behavior_tree.empty()) {
    goal->set_behavior_tree(behavior_tree);
  }
  return impl_->runNavigator(kNavigatorNavigateThroughPoses,
    [&](Impl::CancelFn cancel) {
      auto nav = impl_->get<navigation::NavigateThroughPosesNavigator>(
        kNavigatorNavigateThroughPoses);
      return nav->Run(goal, cancel);
    });
}

behavior_tree::BtStatus NavigatorRegistry::navigateToDock(
  const commsgs::geometry_msgs::PoseStamped & dock_pose,
  const std::string & dock_type, const std::string & dock_id)
{
  auto goal = std::make_shared<proto::DockRobotAction::Goal>();
  goal->set_use_dock_id(!dock_id.empty());
  goal->set_dock_id(dock_id);
  goal->set_dock_type(dock_type);
  goal->set_navigate_to_staging_pose(true);
  *goal->mutable_dock_pose() = commsgs::geometry_msgs::ToProto(dock_pose);
  return impl_->runNavigator(kNavigatorNavigateToDocking,
    [&](Impl::CancelFn cancel) {
      auto nav = impl_->get<docking::NavigateToDockingNavigator>(
        kNavigatorNavigateToDocking);
      return nav->Run(goal, cancel);
    });
}

behavior_tree::BtStatus NavigatorRegistry::trackToTarget(const uint32_t target_id)
{
  auto goal = std::make_shared<proto::TrackToTargetAction::Goal>();
  goal->set_target_id(target_id);
  return impl_->runNavigator(kNavigatorTrackToTarget,
    [&](Impl::CancelFn cancel) {
      auto nav = impl_->get<tracking::TrackToTargetNavigator>(
        kNavigatorTrackToTarget);
      return nav->Run(goal, cancel);
    });
}

behavior_tree::BtStatus NavigatorRegistry::exploreAnywhere(
  const double time_allowance_sec)
{
  auto goal = std::make_shared<
    proto::ExploreToAnywhereAction::Goal>();
  if (time_allowance_sec > 0.0) {
    *goal->mutable_time_allowance() = commsgs::builtin_interfaces::ToProto(
      commsgs::builtin_interfaces::Duration::FromSeconds(time_allowance_sec));
  }
  return impl_->runNavigator(kNavigatorExploreToAnywhere,
    [&](Impl::CancelFn cancel) {
      auto nav = impl_->get<exploration::ExploreToAnywhereNavigator>(
        kNavigatorExploreToAnywhere);
      return nav->Run(goal, cancel);
    });
}

behavior_tree::BtStatus NavigatorRegistry::teleopDrive(
  const double time_allowance_sec,
  const double max_linear_vel,
  const double max_angular_vel,
  std::function<bool()> cancel_checker)
{
  auto goal = std::make_shared<proto::AssistedTeleopAction::Goal>();
  if (time_allowance_sec > 0.0) {
    *goal->mutable_time_allowance() = commsgs::builtin_interfaces::ToProto(
      commsgs::builtin_interfaces::Duration::FromSeconds(time_allowance_sec));
  }
  return impl_->runNavigator(kNavigatorTeleopDrive,
    [&, checker = std::move(cancel_checker)](Impl::CancelFn cancel) {
      auto nav = impl_->get<teleop::TeleopDriveNavigator>(kNavigatorTeleopDrive);
      if (!nav) {
        return behavior_tree::BtStatus::FAILED;
      }
      nav->setRunLimits(max_linear_vel, max_angular_vel);
      Impl::CancelFn merged_cancel = [cancel, &checker]() {
        return cancel() || (checker && checker());
      };
      return nav->Run(goal, merged_cancel);
    });
}

teleop::TeleopSession * NavigatorRegistry::teleopSession()
{
  return const_cast<teleop::TeleopSession *>(impl_->teleopSession());
}

const teleop::TeleopSession * NavigatorRegistry::teleopSession() const
{
  return impl_->teleopSession();
}

bool NavigatorRegistry::updateTrackTargetPose(
  const commsgs::geometry_msgs::PoseStamped & target_pose)
{
  auto nav = impl_->get<tracking::TrackToTargetNavigator>(kNavigatorTrackToTarget);
  if (!nav) {
    return false;
  }
  nav->UpdateTargetPose(target_pose);
  return true;
}

bool NavigatorRegistry::updateExploreGoal(
  const commsgs::geometry_msgs::PoseStamped & explore_goal)
{
  auto nav = impl_->get<exploration::ExploreToAnywhereNavigator>(
    kNavigatorExploreToAnywhere);
  if (!nav) {
    return false;
  }
  nav->UpdateExploreGoal(explore_goal);
  return true;
}

bool NavigatorRegistry::hasNavigator(const std::string & id) const
{
  return impl_->navigators.find(id) != impl_->navigators.end();
}

std::vector<std::string> NavigatorRegistry::registeredNavigatorIds() const
{
  std::vector<std::string> ids;
  ids.reserve(impl_->navigators.size());
  for (const auto & entry : impl_->navigators) {
    ids.push_back(entry.first);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

const proto::TaskOptions & NavigatorRegistry::taskOptions() const
{
  return impl_->task_options_;
}

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
