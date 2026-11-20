/* Copyright 2026 The Openbot Authors. */
/** @file native_vista_path_provider.hpp
 * @brief Exploration routes from the PlannerServer-owned graph over native Autolink.
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include "autonomy/task/exploration/integration/path_provider.hpp"

namespace autolink { class Node; }
namespace autonomy::task::exploration {

/** @brief Endpoint, frame and bounded response settings for one route client. */
struct NativeVistaPathOptions {
    std::string service_name = "/get_plan";  ///< Existing automsgs route service.
    std::string planner_id = "vista_planner";  ///< Exact configured graph owner id.
    std::string frame_id = "map";  ///< Frame shared with localization and mapping.
    std::string navigation_tree;  ///< Optional exploration navigation BT; empty uses the task default.
    std::chrono::milliseconds timeout{500};  ///< Includes client serialization and reply wait.
    std::chrono::milliseconds retry_interval{100};  ///< Same read-only request/slot; zero disables retransmission.
    std::size_t maximum_poses = 65536;  ///< Bound accepted path memory and validation work.
};

/**
 * @brief Read-only cross-process adapter; never constructs a competing graph.
 * @details One outstanding request is retained until its actual completion. A
 * timeout/cancel leaves that slot occupied, so retries cannot grow the native
 * client's pending-request map. Late replies are discarded before another send.
 * While waiting within the original deadline, idempotent queries can retransmit
 * the same native request identity to survive late receiver discovery.
 * Transport faults throw from Route rather than certify geometric unreachability.
 * Cancellation stops local waiting; the server has an independent work budget.
 */
class NativeVistaPathProvider final : public PathProvider {
public:
    /** @brief Create a native client on an existing task-owned communication node.
     * @throws std::invalid_argument Empty identity, invalid capacity or timeout.
     * @throws std::runtime_error The actual native client could not be initialized.
     */
    explicit NativeVistaPathProvider(std::shared_ptr<autolink::Node> node,
                                    NativeVistaPathOptions options = {});
    /** @brief Release the native client before releasing its owning node. */
    ~NativeVistaPathProvider() override;
    /** @brief Query fresh owner evidence; absence cannot satisfy task completion. */
    std::optional<RouteMapEvidence> Evidence() const override;
    /** @brief Query exact endpoints, retaining final observation yaw and route provenance. */
    planning::planner::vista::VistaRoute Route(const Pose2d& start,
                                              const Pose2d& goal) const override;
    /** @brief Cancel locally and reject stale or malformed native replies.
     * @throws std::runtime_error Transport, protocol, configuration or source failure.
     */
    planning::planner::vista::VistaRoute Route(const Pose2d& start, const Pose2d& goal,
                                              const std::function<bool()>& canceled) const override;
private:
    class Impl;  ///< Owns actual transport and serializes one outstanding request.
    std::unique_ptr<Impl> impl_;  ///< Out-of-line native transport lifetime.
};

}  // namespace autonomy::task::exploration
