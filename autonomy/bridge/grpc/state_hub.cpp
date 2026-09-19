/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file state_hub.cpp
 * @brief Implementation of StateHub reader callback and GetSnapshot overlay.
 */

#include "autonomy/bridge/grpc/state_hub.hpp"

#include "autonomy/bridge/constants.hpp"
#include "autolink/common/log.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

StateHub::StateHub(std::shared_ptr<autolink::Node> node,
                   TaskMuxer::SharedPtr muxer)
    : node_(std::move(node)), muxer_(std::move(muxer)) {
    if (!node_) {
        return;
    }
    state_reader_ =
        node_->CreateReader<::automsgs::msgs::vehicle_msgs::RobotState>(
            kRobotStateChannel,
            [this](const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>&
                       message) { HandleState(message); });
    if (!state_reader_) {
        AWARN << "StateHub: no reader on " << kRobotStateChannel;
    }
}

::automsgs::msgs::vehicle_msgs::RobotState StateHub::SynthesizeLocked() const {
    ::automsgs::msgs::vehicle_msgs::RobotState state = latest_;
    if (muxer_) {
        const auto snapshot = muxer_->GetSnapshot();
        state.set_active_task_type(snapshot.type);
        state.set_active_task_status(snapshot.status);
        state.set_active_cmd_id(snapshot.cmd_id);
        state.set_motion_enabled(!muxer_->IsEstop());
    }
    return state;
}

::automsgs::msgs::vehicle_msgs::RobotState StateHub::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return SynthesizeLocked();
}

void StateHub::HandleState(
    const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>& message) {
    if (!message) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = *message;
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
