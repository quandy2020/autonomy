/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/node_client.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/rpcs/teleop.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * Relative teleop: DriveOnHeading / BackUp / Spin via controller actions.
 */
class RelativeTeleopStub
{
public:
    using StreamCallback = std::function<void(
        const ::automsgs::rpcs::teleop::TeleopResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(RelativeTeleopStub)

    RelativeTeleopStub(std::shared_ptr<autolink::Node> node,
                       std::shared_ptr<TaskMuxer> muxer);

    bool DriveOnHeading(
        const ::automsgs::rpcs::teleop::DriveOnHeadingRequest& request,
        StreamCallback callback);
    bool BackUp(const ::automsgs::rpcs::teleop::BackUpRequest& request,
                StreamCallback callback);
    bool Spin(const ::automsgs::rpcs::teleop::SpinRequest& request,
              StreamCallback callback);
    bool Cancel(const std::string& goal_id);
    bool Pause(const std::string& goal_id);
    bool Resume(const std::string& goal_id);
    ::automsgs::rpcs::teleop::TeleopResponse GetSnapshot() const;

private:
    enum class Mode { kNone, kDrive, kBackUp, kSpin };

    using DriveClient =
        NodeClient<::automsgs::actions::DriveOnHeadingAction>;
    using BackUpClient = NodeClient<::automsgs::actions::BackUpAction>;
    using SpinClient = NodeClient<::automsgs::actions::SpinAction>;

    void ClearLockedState();
    ::automsgs::rpcs::teleop::TeleopResponse MakeResponse(
        const std::string& goal_id, ::automsgs::rpcs::teleop::TeleopState state,
        bool ok, const std::string& detail = "") const;

    DriveClient::SharedPtr drive_client_;
    BackUpClient::SharedPtr backup_client_;
    SpinClient::SharedPtr spin_client_;
    std::shared_ptr<TaskMuxer> muxer_;

    mutable std::mutex mutex_;
    Mode mode_{Mode::kNone};
    std::string goal_id_;
    bool paused_{false};
    ::automsgs::rpcs::teleop::TeleopState state_{
        ::automsgs::rpcs::teleop::TELEOP_STATE_IDLE};
    std::shared_ptr<DriveClient::GoalHandle> drive_handle_;
    std::shared_ptr<BackUpClient::GoalHandle> backup_handle_;
    std::shared_ptr<SpinClient::GoalHandle> spin_handle_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
