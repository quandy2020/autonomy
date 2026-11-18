/* Copyright 2026 The Openbot Authors. */
/** @file exploration_session.hpp
 * @brief Native single-owner exploration session and asynchronous planning input.
 */
#pragma once

#include <memory>
#include "autonomy/perception/observation/observation_ingress.hpp"
#include "autonomy/task/exploration/exploration_context.hpp"
#include "autonomy/task/exploration/strategy/exploration_strategy.hpp"

namespace autonomy::task::exploration {

/** @brief Owns exploration evidence, phases and versioned asynchronous requests.
 * Input callbacks enqueue data. Tick has one owner; a dedicated worker plans
 * against immutable snapshots. Requests bind finite admitted input prefixes,
 * preserving progress under continuous sensor production.
 */
class ExplorationSession {
public:
    /** @brief Creates an owner and worker with verified integration dependencies. */
    ExplorationSession(std::shared_ptr<ExplorationStrategy> strategy,
        std::shared_ptr<const GoalPoseAdapter> adapter,
        std::shared_ptr<const PathProvider> paths, ExplorationServices services,
        ExplorationSessionOptions options = {});
    /** @brief Cancels outstanding work and releases the worker and services. */
    ~ExplorationSession();
    ExplorationSession(const ExplorationSession&) = delete;
    ExplorationSession& operator=(const ExplorationSession&) = delete;

    /** @brief Starts a new session and resets prior request/observation baselines. */
    void Start(const Pose2d& home, std::uint64_t map_epoch, double now);
    /** @brief Enqueues an immutable map consumer view, respecting admission capacity. */
    bool PushMap(MapReadView map);
    /** @brief Enqueues timestamped localization evidence in the named map epoch. */
    bool PushPose(Pose2d pose, double stamp, std::uint64_t map_epoch,
                  perception::SensorMask localization_support);
    /** @brief Admits a measured source input; rejection does not count as coverage. */
    bool PushObservation(perception::SensorObservation observation, double now);
    /** @brief Atomically queue one acquisition-correlated pose/observation input.
     * @return False for invalid evidence, source policy or bounded queue rejection.
     * @details Owner processing preserves actual beam poses and records expiry/
     *          overload losses; producer callbacks never apply evidence or tick.
     */
    bool PushSensorPacket(perception::ObservationPacket packet, double now);
    /** @brief Retain a rejected receiver ordinal as loss, never as processed coverage. */
    void RejectSensorInput(perception::SensorKind sensor, std::uint64_t sequence);
    /** @brief Read at most four admitted, unexpired depth inputs for DeFM.
     * Returned immutable views preserve the original calibration and pose.
     * This is raw input availability, not a learned-controller validation.
     */
    std::vector<std::shared_ptr<const perception::LocalizedDepthView>> RecentDepthViews(double now) const;
    /** @brief Enqueues an explicit requirement update, never predicted evidence. */
    bool PushCoverageUnit(CoverageUnit unit);
    /** @brief Posts pause/resume/cancel on the priority control queue. */
    bool PostCommand(SessionCommand command);
    /** @brief Posts a cancellation for one matching versioned request. */
    bool CancelRequest(const RequestVersion& version);
    // Requests originate on the same owner thread as Tick. One phase owns the
    // in-flight work; repeated BT ticks query the ticket rather than resubmit.
    /** @brief Requests one BT phase on the Tick owner without waiting for all input. */
    std::optional<PhaseTicket> RequestPhase(ExplorationPhase phase, double now);
    /** @brief Reads a ticket's authoritative owner-published result. */
    PhaseResult PhaseState(const PhaseTicket& ticket) const;
    /** @brief Enqueues cancellation of a matching phase ticket. */
    bool CancelPhase(const PhaseTicket& ticket);
    /** @brief Returns the configured automatic or BT driver. */
    ExplorationDriver driver() const;
    /** @brief Advances bounded input processing and asynchronous owner state. */
    ExplorationSnapshot Tick(double now);
    /** @brief Returns a synchronized immutable copy of published owner state. */
    ExplorationSnapshot Snapshot() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;  ///< Owner, input buffers, services and planning worker.
};
}  // namespace autonomy::task::exploration
