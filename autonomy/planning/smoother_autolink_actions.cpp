/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/smoother_server.hpp"

#include <memory>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"

namespace autonomy {
namespace planning {
namespace {

namespace task_proto = tasks::behavior_tree::task_proto;
using SmoothPathServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::SmoothPathActionTraits>;

}  // namespace

struct SmootherServer::AutolinkActionServers {
    std::shared_ptr<SmoothPathServer> smooth_path;
};

bool SmootherServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    if (!node || autolink_actions_) {
        return node != nullptr;
    }
    autolink_actions_ = new AutolinkActionServers();
    SmootherServer* self = this;

    autolink_actions_->smooth_path = std::make_shared<SmoothPathServer>(
        node, tasks::behavior_tree::kSmoothPathActionName, [self]() {
            auto server = self->autolink_actions_->smooth_path;
            auto goal = server->GetCurrentGoal();
            if (!goal || !goal->has_path()) {
                auto result =
                    std::make_shared<task_proto::SmoothPathAction_Result>();
                result->set_error_code(task_proto::SMOOTH_PATH_UNKNOWN);
                server->TerminateCurrent(result);
                return;
            }
            auto path = commsgs::planning_msgs::FromProto(goal->path());
            std::chrono::milliseconds max_time{1000};
            if (goal->has_max_smoothing_duration()) {
                const double max_sec =
                    commsgs::builtin_interfaces::FromProto(
                        goal->max_smoothing_duration())
                        .Seconds();
                if (max_sec > 0.0) {
                    max_time = std::chrono::milliseconds(static_cast<int>(
                        max_sec * 1000.0));
                }
            }
            auto cancel = [&]() { return server->IsCancelRequested(); };
            try {
                auto out = self->SmoothPath(
                    path, goal->smoother_id(), max_time,
                    goal->check_for_collisions(), cancel);
                auto result =
                    std::make_shared<task_proto::SmoothPathAction_Result>();
                *result->mutable_path() = commsgs::planning_msgs::ToProto(out.path);
                result->set_was_completed(out.was_completed);
                result->set_error_code(task_proto::SMOOTH_PATH_NONE);
                server->SucceededCurrent(result);
            } catch (const common::SmootherException& ex) {
                auto result =
                    std::make_shared<task_proto::SmoothPathAction_Result>();
                result->set_error_code(task_proto::SMOOTH_PATH_FAILED_TO_SMOOTH);
                result->set_error_msg(ex.what());
                server->TerminateCurrent(result);
            }
        });

    AINFO << "SmootherServer autolink actions attached.";
    return true;
}

SmootherServer::~SmootherServer() {
    DetachAutolinkNode();
    Shutdown();
}

void SmootherServer::DetachAutolinkNode() {
    if (autolink_actions_) {
        if (autolink_actions_->smooth_path) {
            autolink_actions_->smooth_path->Deactivate();
        }
        delete autolink_actions_;
        autolink_actions_ = nullptr;
    }
}

}  // namespace planning
}  // namespace autonomy
