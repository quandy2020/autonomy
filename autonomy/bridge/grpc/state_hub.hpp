/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file state_hub.hpp
 * @brief Caches `/robot_state` and overlays muxer task fields for System RPCs.
 *
 * @details
 * Subscribes to @c kRobotStateChannel (`/robot_state` in bridge/constants.hpp)
 * via Autolink, keeps the latest `vehicle_msgs.RobotState` under a mutex, and
 * applies TaskMuxer snapshot overlays when serving System / profile readers.
 * When no live sample has arrived, SynthesizeLocked builds a minimal state
 * from muxer fields alone (so GetSnapshot never blocks for the first message).
 *
 * Related topics (not subscribed here):
 * - @c kRobotEventChannel (`/robot_event`) — events are out of scope for Hub
 *
 * Invariants:
 * - Reader updates under mutex_; GetSnapshot returns a value copy (never a
 *   reference into latest_).
 * - Muxer overlay is applied at read time; Hub never calls CancelAll / Estop.
 * - Push fan-out was removed with AutonomyService ReceiveBot*.
 *
 * Ownership: Context owns UniquePtr; handlers / profile helpers reach it via
 * Context::state_hub(). Muxer SharedPtr is retained so overlays remain valid
 * while Hub and stubs share the same TaskMuxer.
 *
 * Threading: HandleState runs on the Autolink reader callback thread;
 * GetSnapshot may run on gRPC handler threads. Both take mutex_.
 *
 * @see TaskMuxer
 * @see profile.hpp
 * @see rpc_system_handlers.hpp
 */

#pragma once

#include <memory>
#include <mutex>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Caches `/robot_state` and overlays muxer task fields for System RPCs.
 *
 * @details Owns an Autolink Reader for RobotState and a SharedPtr TaskMuxer
 * for synthesized / overlaid motion and task flags. Primary consumer is
 * SystemService + BuildRpcRobotFullInfo.
 *
 * @note GetSnapshot is const and thread-safe under mutex_.
 * @warning SynthesizeLocked must only be called with mutex_ held.
 * @see kRobotStateChannel
 */
class StateHub
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(StateHub)

    /**
     * @brief Construct a reader for the robot-state topic.
     *
     * Creates the Autolink Reader and binds HandleState. Does not block for
     * the first sample — GetSnapshot may synthesize until one arrives.
     *
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer (for synthesized motion / task flags).
     */
    StateHub(std::shared_ptr<autolink::Node> node, TaskMuxer::SharedPtr muxer);

    /**
     * @brief Return the latest robot-state snapshot.
     *
     * Copies latest_ (or SynthesizeLocked()) under mutex_ and applies muxer
     * overlay fields as implemented in the .cc.
     *
     * @return Cached or synthesized RobotState value.
     */
    ::automsgs::msgs::vehicle_msgs::RobotState GetSnapshot() const;

private:
    /**
     * @brief Handle an incoming robot-state message.
     *
     * Replaces latest_ under mutex_. Ignores null @p message.
     *
     * @param[in] message Shared RobotState sample from Autolink.
     */
    void HandleState(const std::shared_ptr<::automsgs::msgs::vehicle_msgs::RobotState>& message);

    /**
     * @brief Build a synthesized state when no live sample exists.
     *
     * @return Synthesized RobotState (caller must hold mutex_).
     *
     * @warning Must only be called with mutex_ held.
     */
    ::automsgs::msgs::vehicle_msgs::RobotState SynthesizeLocked() const;

    /**
     * @brief Autolink node used to create the robot-state Reader.
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Shared TaskMuxer for overlay / synthesize of active-task fields.
     */
    TaskMuxer::SharedPtr muxer_{nullptr};

    /**
     * @brief Autolink Reader subscribed to @c kRobotStateChannel.
     */
    std::shared_ptr<autolink::Reader<::automsgs::msgs::vehicle_msgs::RobotState>> state_reader_{nullptr};

    /**
     * @brief Guards latest_ against concurrent HandleState / GetSnapshot.
     */
    mutable std::mutex mutex_;

    /**
     * @brief Latest RobotState sample (empty / default until first message).
     */
    ::automsgs::msgs::vehicle_msgs::RobotState latest_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
