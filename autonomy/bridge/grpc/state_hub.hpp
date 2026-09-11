/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_event.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Caches `/robot_state` and fans out `/robot_event` for push RPCs.
 */
class StateHub
{
public:
    using StateCallback =
        std::function<void(const ::automsgs::msgs::vehicle_msgs::RobotState&)>;
    using EventCallback =
        std::function<void(const ::automsgs::msgs::vehicle_msgs::RobotEvent&)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(StateHub)

    /**
     * @brief Construct readers for robot state / event topics.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer (for synthesized motion flags).
     */
    StateHub(std::shared_ptr<autolink::Node> node,
             std::shared_ptr<TaskMuxer> muxer);

    /** @brief Return the latest robot-state snapshot. */
    ::automsgs::msgs::vehicle_msgs::RobotState GetSnapshot() const;

    /**
     * @brief Subscribe to robot-state updates.
     * @param[in] callback State callback.
     * @return Subscription id for `Unsubscribe`.
     */
    int SubscribeState(StateCallback callback);

    /**
     * @brief Subscribe to robot-event updates.
     * @param[in] callback Event callback.
     * @return Subscription id for `Unsubscribe`.
     */
    int SubscribeEvent(EventCallback callback);

    /**
     * @brief Remove a state or event subscription.
     * @param[in] id Subscription id returned by Subscribe*.
     */
    void Unsubscribe(int id);

private:
    /** @brief Handle an incoming robot-state message. */
    void HandleState(
        const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>& message);
    /** @brief Handle an incoming robot-event message. */
    void HandleEvent(
        const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotEvent>& message);
    /** @brief Build a synthesized state when no live sample exists. */
    ::automsgs::msgs::vehicle_msgs::RobotState SynthesizeLocked() const;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<TaskMuxer> muxer_;
    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::vehicle_msgs::RobotState>>
        state_reader_;
    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::vehicle_msgs::RobotEvent>>
        event_reader_;

    mutable std::mutex mutex_;
    ::automsgs::msgs::vehicle_msgs::RobotState latest_;
    bool has_state_{false};
    int next_id_{1};
    std::unordered_map<int, StateCallback> state_subs_;
    std::unordered_map<int, EventCallback> event_subs_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
