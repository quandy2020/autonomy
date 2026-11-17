/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <memory>
#include "autonomy/task/exploration/exploration_context.hpp"
#include "autonomy/task/exploration/strategy/exploration_strategy.hpp"

namespace autonomy::task::exploration {

// Input callbacks enqueue data. Tick is called by one task owner; planning runs
// on a dedicated worker and returns immutable, versioned results to that owner.
class ExplorationSession {
public:
    ExplorationSession(std::shared_ptr<ExplorationStrategy> strategy,
        std::shared_ptr<const GoalPoseAdapter> adapter,
        std::shared_ptr<const PathProvider> paths, ExplorationServices services,
        ExplorationSessionOptions options = {});
    ~ExplorationSession();
    ExplorationSession(const ExplorationSession&) = delete;
    ExplorationSession& operator=(const ExplorationSession&) = delete;

    void Start(const Pose2d& home, std::uint64_t map_epoch, double now);
    bool PushMap(MapReadView map);
    bool PushPose(Pose2d pose, double stamp, std::uint64_t map_epoch,
                  perception::SensorMask localization_support);
    bool PushObservation(perception::SensorObservation observation, double now);
    bool PushCoverageUnit(CoverageUnit unit);
    bool PostCommand(SessionCommand command);
    bool CancelRequest(const RequestVersion& version);
    // Requests originate on the same owner thread as Tick. One phase owns the
    // in-flight work; repeated BT ticks query the ticket rather than resubmit.
    std::optional<PhaseTicket> RequestPhase(ExplorationPhase phase, double now);
    PhaseResult PhaseState(const PhaseTicket& ticket) const;
    bool CancelPhase(const PhaseTicket& ticket);
    ExplorationDriver driver() const;
    ExplorationSnapshot Tick(double now);
    ExplorationSnapshot Snapshot() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace autonomy::task::exploration
