/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigator/navigator_factory.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/navigator/docking/navigate_to_docking.hpp"
#include "autonomy/tasks/navigator/exploration/explore_to_anywhere.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_through_poses.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/navigator/tracking/track_to_target.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace {

std::string DefaultBehaviorTreeFile(const proto::NavigatorConfig& config,
                                    const char* fallback) {
    return config.default_behavior_tree_file().empty()
               ? fallback
               : config.default_behavior_tree_file();
}

common::FeedbackUtils WithDefaultBt(common::FeedbackUtils feedback,
                                    const proto::NavigatorConfig& config,
                                    const char* fallback) {
    feedback.default_bt_xml_filename =
        DefaultBehaviorTreeFile(config, fallback);
    return feedback;
}

template <typename NavigatorT>
std::unique_ptr<common::NavigatorBase> MakeNavigator(
    const NavigatorCreateContext& ctx, const proto::NavigatorConfig& config,
    const char* default_bt) {
    auto feedback = WithDefaultBt(ctx.feedback_utils, config, default_bt);
    return std::make_unique<NavigatorT>(
        ctx.options, ctx.task_context, ctx.plugin_lib_names, feedback,
        ctx.muxer, ctx.odom_smoother);
}

}  // namespace

NavigatorFactoryRegistry& NavigatorFactory::RegistryInstance() {
    static NavigatorFactoryRegistry registry;
    return registry;
}

void NavigatorFactory::EnsureBuiltinsRegistered() {
    static bool registered = false;
    if (registered) {
        return;
    }
    registered = RegisterBuiltinNavigators(RegistryInstance());
}

bool NavigatorFactory::HasNavigator(const std::string& id) {
    EnsureBuiltinsRegistered();
    return RegistryInstance().Contains(id);
}

std::unique_ptr<common::NavigatorBase> NavigatorFactory::Create(
    const std::string& id, const NavigatorCreateContext& context,
    const proto::NavigatorConfig& config) {
    EnsureBuiltinsRegistered();
    return RegistryInstance().CreateObjectOrNull(id, context, config);
}

bool RegisterBuiltinNavigators(NavigatorFactoryRegistry& factory) {
    bool ok = true;
    ok &= factory.Register(
        "navigate_to_pose",
        [](const NavigatorCreateContext& ctx,
           const proto::NavigatorConfig& config) {
            return MakeNavigator<navigation::NavigateToPoseNavigator>(
                ctx, config, "navigate_to_pose.xml");
        });
    ok &= factory.Register(
        "navigate_through_poses",
        [](const NavigatorCreateContext& ctx,
           const proto::NavigatorConfig& config) {
            return MakeNavigator<navigation::NavigateThroughPosesNavigator>(
                ctx, config, "navigate_through_poses.xml");
        });
    ok &= factory.Register(
        "navigate_to_docking",
        [](const NavigatorCreateContext& ctx,
           const proto::NavigatorConfig& config) {
            return MakeNavigator<docking::NavigateToDockingNavigator>(
                ctx, config, "navigate_to_dock.xml");
        });
    ok &= factory.Register(
        "track_to_target",
        [](const NavigatorCreateContext& ctx,
           const proto::NavigatorConfig& config) {
            return MakeNavigator<tracking::TrackToTargetNavigator>(
                ctx, config, "track_to_target.xml");
        });
    ok &= factory.Register(
        "explore_to_anywhere",
        [](const NavigatorCreateContext& ctx,
           const proto::NavigatorConfig& config) {
            return MakeNavigator<exploration::ExploreToAnywhereNavigator>(
                ctx, config, "explore_to_anywhere.xml");
        });
    if (!ok) {
        AERROR << "Failed to register one or more builtin BT navigators.";
    }
    return ok;
}

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
