/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/common/navigation_engine.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/navigator/bt_navigator.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {
namespace {

class StubNavigationEngine : public NavigationEngine
{
public:
    bool Configure(
        const proto::TaskOptions&,
        std::shared_ptr<planning::PlannerServer>,
        std::shared_ptr<planning::SmootherServer>,
        std::shared_ptr<control::ControllerServer>,
        std::shared_ptr<transform::Buffer>) override {
        configured_ = true;
        return true;
    }

    bool StartNavigateToPose(const commsgs::geometry_msgs::PoseStamped&,
                             const std::string&) override {
        AWARN << "NavigationEngine stub: BtNavigator not linked; cannot start "
                 "navigate_to_pose.";
        return false;
    }

    bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>&,
        const std::string&) override {
        AWARN << "NavigationEngine stub: BtNavigator not linked; cannot start "
                 "navigate_through_poses.";
        return false;
    }

    bool Cancel() override { return false; }
    bool Pause() override { return false; }
    bool Resume() override { return false; }

    NavigationMode GetActiveMode() const override { return NavigationMode::NONE; }
    bool IsActive() const override { return false; }

private:
    bool configured_{false};
};

}  // namespace

std::shared_ptr<NavigationEngine> CreateStubNavigationEngine() {
    return std::make_shared<StubNavigationEngine>();
}

std::shared_ptr<NavigationEngine> CreateBtNavigationEngine() {
    return std::make_shared<behavior_tree::BtNavigator>();
}

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
