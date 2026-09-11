/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/state_hub.hpp"

#include "autonomy/bridge/constants.hpp"
#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_status.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

::automsgs::msgs::vehicle_msgs::RobotTaskType ToVehicleTaskType(
    proto::TaskType type) {
    return static_cast<::automsgs::msgs::vehicle_msgs::RobotTaskType>(
        static_cast<int>(type));
}

::automsgs::msgs::vehicle_msgs::RobotTaskStatus ToVehicleTaskStatus(
    proto::TaskStatus status) {
    return static_cast<::automsgs::msgs::vehicle_msgs::RobotTaskStatus>(
        static_cast<int>(status));
}

}  // namespace

StateHub::StateHub(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<TaskMuxer> muxer)
    : node_(std::move(node)), muxer_(std::move(muxer)) {
    if (!node_) {
        return;
    }
    state_reader_ =
        node_->CreateReader<::automsgs::msgs::vehicle_msgs::RobotState>(
            kRobotStateChannel,
            [this](const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>&
                       message) { HandleState(message); });
    event_reader_ =
        node_->CreateReader<::automsgs::msgs::vehicle_msgs::RobotEvent>(
            kRobotEventChannel,
            [this](const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotEvent>&
                       message) { HandleEvent(message); });
    if (!state_reader_) {
        AWARN << "StateHub: no reader on " << kRobotStateChannel;
    }
    if (!event_reader_) {
        AWARN << "StateHub: no reader on " << kRobotEventChannel;
    }
}

::automsgs::msgs::vehicle_msgs::RobotState StateHub::SynthesizeLocked() const {
    ::automsgs::msgs::vehicle_msgs::RobotState state = latest_;
    if (muxer_) {
        const auto snapshot = muxer_->GetSnapshot();
        state.set_active_task_type(ToVehicleTaskType(snapshot.type()));
        state.set_active_task_status(ToVehicleTaskStatus(snapshot.status()));
        state.set_active_cmd_id(snapshot.cmd_id());
        state.set_motion_enabled(!muxer_->CheckEstopActive());
    }
    return state;
}

::automsgs::msgs::vehicle_msgs::RobotState StateHub::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return SynthesizeLocked();
}

int StateHub::SubscribeState(StateCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int id = next_id_++;
    state_subs_[id] = std::move(callback);
    return id;
}

int StateHub::SubscribeEvent(EventCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int id = next_id_++;
    event_subs_[id] = std::move(callback);
    return id;
}

void StateHub::Unsubscribe(const int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_subs_.erase(id);
    event_subs_.erase(id);
}

void StateHub::HandleState(
    const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>& message) {
    if (!message) {
        return;
    }
    std::vector<StateCallback> callbacks;
    ::automsgs::msgs::vehicle_msgs::RobotState synthesized;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = *message;
        has_state_ = true;
        synthesized = SynthesizeLocked();
        callbacks.reserve(state_subs_.size());
        for (const auto& entry : state_subs_) {
            callbacks.push_back(entry.second);
        }
    }
    for (const auto& callback : callbacks) {
        callback(synthesized);
    }
}

void StateHub::HandleEvent(
    const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotEvent>& message) {
    if (!message) {
        return;
    }
    std::vector<EventCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks.reserve(event_subs_.size());
        for (const auto& entry : event_subs_) {
            callbacks.push_back(entry.second);
        }
    }
    for (const auto& callback : callbacks) {
        callback(*message);
    }
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
