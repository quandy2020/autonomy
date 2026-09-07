/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/tracking/tracking_client.hpp"

#include <automsgs/actions/nav_actions.pb.h>

#include <exception>
#include <optional>
#include <string>
#include <utility>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/tracking/plugins/plugin_utils.hpp"

namespace autonomy::task::tracking::internal {

ShadowPathExecutor::ShadowPathExecutor(Operations operations)
    : operations_(std::move(operations)) {}

BT::NodeStatus ShadowPathExecutor::Tick(const std::string& controller_id,
                                        int* error_code,
                                        std::string* error_message) {
    if (!operations_.get_path || !operations_.action_ready ||
        !operations_.begin || !operations_.tick || !operations_.cancel) {
        Fail(-1, "FollowShadowPath: incomplete action session", error_code,
             error_message);
        return BT::NodeStatus::FAILURE;
    }

    try {
        Path path;
        uint64_t revision = 0;
        if (!operations_.get_path(&path, &revision) || path.poses().empty()) {
            Fail(1, "FollowShadowPath: path unavailable or stale", error_code,
                 error_message);
            return BT::NodeStatus::FAILURE;
        }
        if (!active_ && has_completed_revision_ &&
            revision == completed_revision_) {
            return BT::NodeStatus::RUNNING;
        }
        if (!operations_.action_ready()) {
            Fail(1, "FollowShadowPath: follow_path server not ready",
                 error_code, error_message);
            return BT::NodeStatus::FAILURE;
        }

        if (!has_active_revision_ || revision != active_revision_) {
            operations_.begin(path, controller_id);
            active_revision_ = revision;
            has_active_revision_ = true;
            has_completed_revision_ = false;
            active_ = true;
            return BT::NodeStatus::RUNNING;
        }

        const BT::NodeStatus status =
            operations_.tick(error_code, error_message);
        if (status == BT::NodeStatus::SUCCESS) {
            active_ = false;
            has_active_revision_ = false;
            completed_revision_ = active_revision_;
            has_completed_revision_ = true;
            return BT::NodeStatus::RUNNING;
        }
        if (status == BT::NodeStatus::FAILURE) {
            active_ = false;
            has_active_revision_ = false;
        }
        return status;
    } catch (const std::exception& exception) {
        Fail(-1, exception.what(), error_code, error_message);
        return BT::NodeStatus::FAILURE;
    }
}

void ShadowPathExecutor::Halt() {
    if (operations_.cancel) {
        operations_.cancel();
    }
    active_ = false;
    has_active_revision_ = false;
    has_completed_revision_ = false;
}

void ShadowPathExecutor::Fail(int code, const std::string& message,
                              int* error_code, std::string* error_message) {
    if (operations_.cancel) {
        operations_.cancel();
    }
    active_ = false;
    has_active_revision_ = false;
    has_completed_revision_ = false;
    if (error_code != nullptr) {
        *error_code = code;
    }
    if (error_message != nullptr) {
        *error_message = message;
    }
}

}  // namespace autonomy::task::tracking::internal

namespace autonomy::task::plugins::tracking {

namespace navigation_actions = automsgs::actions;

class FollowShadowPathAction : public BtStatefulAction
{
public:
    FollowShadowPathAction(const std::string& name,
                           const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("controller_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnFirstTick() override {
        const auto client = ResolveClient(*this);
        if (!client) {
            SetErrorPorts(*this, 1,
                          "FollowShadowPath: tracking client missing");
            return BT::NodeStatus::FAILURE;
        }

        const auto navigation = client->navigation_client();
        ::autonomy::task::tracking::internal::ShadowPathExecutor::Operations
            operations{
                [client](automsgs::msgs::nav_msgs::Path* path,
                         uint64_t* revision) {
                    return client->GetShadowPath(path, revision);
                },
                [navigation]() {
                    return navigation && navigation->follow_path_client()
                                             .ActionServerIsReady();
                },
                [navigation](const automsgs::msgs::nav_msgs::Path& path,
                             const std::string& controller_id) {
                    navigation_actions::FollowPathAction::Goal goal;
                    *goal.mutable_path() = path;
                    if (!controller_id.empty()) {
                        goal.set_controller_id(controller_id);
                    }
                    navigation->follow_session().Begin(
                        navigation->follow_path_client(), goal);
                },
                [this, navigation](int* error_code,
                                   std::string* error_message) {
                    return navigation->follow_session().Tick(
                        HaltCancelChecker(),
                        [error_code, error_message](
                            const navigation_actions::FollowPathAction::Result&
                                result) {
                            if (error_code != nullptr) {
                                *error_code =
                                    static_cast<int>(result.error_code());
                            }
                            if (error_message != nullptr) {
                                *error_message = result.error_msg();
                            }
                        },
                        [error_code, error_message](
                            int code, const std::string& message) {
                            if (error_code != nullptr) {
                                *error_code = code;
                            }
                            if (error_message != nullptr) {
                                *error_message = message;
                            }
                        });
                },
                [navigation]() {
                    if (navigation) {
                        navigation->CancelActiveMotion();
                    }
                },
            };
        executor_.emplace(std::move(operations));
        return TickExecutor();
    }

    BT::NodeStatus OnExecute() override {
        return TickExecutor();
    }

    void OnHalted() override {
        if (executor_) {
            executor_->Halt();
        }
    }

private:
    BT::NodeStatus TickExecutor() {
        if (!executor_) {
            SetErrorPorts(*this, 1, "FollowShadowPath: executor unavailable");
            return BT::NodeStatus::FAILURE;
        }

        std::string controller_id;
        (void)getInput("controller_id", controller_id);
        int error_code = 0;
        std::string error_message;
        const BT::NodeStatus status =
            executor_->Tick(controller_id, &error_code, &error_message);
        if (status == BT::NodeStatus::SUCCESS) {
            setOutput("error_code_id", error_code);
            setOutput("error_msg", error_message);
        } else if (status == BT::NodeStatus::FAILURE) {
            SetErrorPorts(*this, error_code, error_message);
        }
        return status;
    }

    std::optional<::autonomy::task::tracking::internal::ShadowPathExecutor>
        executor_;
};

}  // namespace autonomy::task::plugins::tracking

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::task::plugins::tracking::FollowShadowPathAction>(
        "FollowShadowPath");
}
