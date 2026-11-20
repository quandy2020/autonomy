/* Copyright 2026 The Openbot Authors. */
/** @file hierarchical_coverage_strategy.hpp
 * @brief Global region ordering and resumable local/fair coverage enumeration.
 */
#pragma once
#include "autonomy/task/exploration/strategy/exploration_strategy.hpp"
#include "autonomy/task/exploration/frontier/frontier_queue.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint_sampler.hpp"
namespace autonomy::task::exploration {
/** @brief Coverage selection costs, capabilities and per-page work limits. */
struct HierarchicalCoverageOptions {
    double region_size = 4;  ///< Global region side length, in meters.
    double local_radius = 5;  ///< Dense local selection radius, in meters.
    double fairness_interval = 10;  ///< Maximum interval before a fair selection, in seconds.
    double sweep_max_age = 30;  ///< Maximum reuse age of a completed proof, in seconds.
    std::size_t local_views = 5;  ///< Maximum viewpoints in a local coverage route.
    perception::SensorMode mode = perception::SensorMode::kRgbdOnly;  ///< Required input mode.
    autonomy::navigation::SafetyOptions safety;  ///< Footprint and geometric clearance policy.
    ViewpointSamplingOptions sampling;  ///< Enumeration and prediction work budgets.
};
/** @brief Keeps sweep evidence separate from monotonic measured-quality changes. */
class HierarchicalCoverageStrategy final : public ExplorationStrategy {
public:
    /** @brief Constructs a strategy using shared real route and visibility providers. */
    HierarchicalCoverageStrategy(std::shared_ptr<const PathProvider> paths,
        std::shared_ptr<const ViewpointEvaluator> evaluator,HierarchicalCoverageOptions options={});
    /** @brief Clears session history, cursors and completed sweep proofs. */
    void Reset() override;
    /** @brief Selects an intent or reports progress of the finite candidate sweep. */
    StrategyDecision Select(const Pose2d&,const Pose2d&,const MapReadView&,const CoverageState&,
        perception::SensorMask,double now,const std::function<bool()>& cancel_checker={}) override;
    /** @brief Exclude a failed viewpoint neighbourhood from the next selections. */
    void MarkFailed(const ObservationGoal&, double now) override;
    /** @brief Returns the strategy-owned fair revisit queue. */
    FrontierQueue& frontier_queue(){return queue_;}
private:
    std::shared_ptr<const PathProvider> paths_;  ///< Route owner used for travel costs.
    std::shared_ptr<const ViewpointEvaluator> evaluator_;  ///< Actual sensor visibility models.
    HierarchicalCoverageOptions options_;  ///< Frozen strategy configuration.
    RegionManager regions_;  ///< Stable global region assignments.
    FrontierQueue queue_;  ///< Fair pending-work and retry ordering.
    std::uint64_t epoch_ = 0;  ///< Coordinate epoch of all sampling history.
    std::uint64_t next_goal_ = 0;  ///< Session-local monotonic observation intent identity.
    double last_fair_selection_ = -1e30;  ///< Last accepted fair selection time.
    std::uint64_t sampling_map_revision_ = 0;  ///< Current enumeration geometry revision.
    std::uint64_t sampling_coverage_revision_ = 0;  ///< Current requirements revision.
    perception::SensorMask sampling_sensors_ = 0;  ///< Current executable sensor capabilities.
    std::size_t normal_cursor_ = 0;  ///< Next coarse candidate slot.
    std::size_t sweep_cursor_ = 0;  ///< Next exhaustive candidate slot.
    std::size_t region_cursor_ = 0;  ///< Next global region representative batch.
    bool sweep_completed_ = false;  ///< A whole pass had compatible dependencies.
    double sweep_completed_at_ = 0;  ///< Owner time when the last valid pass finished.
    std::uint64_t sweep_start_map_revision_ = 0;  ///< Geometry at the start of that pass.
    std::uint64_t sweep_start_coverage_revision_ = 0;  ///< Requirements at pass start.
    perception::SensorMask sweep_start_sensors_ = 0;  ///< Source capabilities at pass start.
    std::optional<std::pair<int, int>> sampling_start_cell_;  ///< Reachability origin dependency.
    struct FailedViewpoint { Pose2d pose; double until = 0; };
    std::vector<FailedViewpoint> failed_viewpoints_;  ///< Bounded retry cooldown.
};
}  // namespace autonomy::task::exploration
