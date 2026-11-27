/* Copyright 2026 The Openbot Authors. */
/** @file viewpoint_sampler.hpp
 * @brief Resumable candidate enumeration with explicit examination budgets.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "autonomy/common/navigation/reachability.hpp"
#include "autonomy/task/exploration/region/region_manager.hpp"
#include "autonomy/task/exploration/viewpoint/rolling_viewpoint_field.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"

namespace autonomy::task::exploration {
/** @brief Work limits and physical cost estimates for one sampling page. */
struct ViewpointSamplingOptions {
    double spacing = .6;  ///< Coarse position spacing, in meters.
    double local_radius =
        5;  ///< Compatibility setting; locality is selected by strategy.
    double nominal_speed = .4;  ///< Translation speed for travel cost, in m/s.
    double angular_speed = .5;  ///< Rotation speed for turn cost, in rad/s.
    double observation_time = .5;  ///< Expected dwell at one view, in seconds.
    double goal_clearance_reserve =
        .05;  ///< Extra footprint margin required for an executable view.
    unsigned yaw_samples =
        8;  ///< Number of uniform executable direction samples.
    std::size_t maximum_candidates =
        512;  ///< Maximum positive-gain views returned.
    std::size_t maximum_evaluations =
        8192;  ///< Maximum visibility predictions.
    std::size_t maximum_examined =
        100000;  ///< Maximum slots checked, including skips.
                 ///< One call must reach the known region
                 ///< instead of ending in empty unknown corners.
    std::optional<perception::Bounds2d>
        allowed_region;  ///< Broad-phase bounds for candidate enumeration.
    std::optional<::automsgs::msgs::geometry_msgs::Polygon>
        allowed_polygon;  ///< Authoritative closed candidate boundary.
    std::vector<::automsgs::msgs::geometry_msgs::Polygon>
        no_go_areas;  ///< Closed candidate exclusion polygons.
};

/** @brief Page progress; a limited or canceled page is never a whole sweep
 * proof. */
struct ViewpointSamplingResult {
    std::vector<Viewpoint>
        candidates;  ///< Reachable views with positive marginal gain.
    bool exhaustive =
        false;  ///< A sweep enumerated the entire domain starting at zero.
    bool limited =
        false;  ///< Work budget ended before the domain was exhausted.
    bool finished =
        false;  ///< This page reached the end of the candidate domain.
    bool canceled = false;  ///< Preparation or enumeration was canceled.
    std::size_t next_cursor =
        0;  ///< First unexamined slot, or domain size at the end.
    std::size_t examined =
        0;  ///< Slots checked, including nonfree and unreachable ones.
    std::size_t evaluated = 0;  ///< Calls made to the visibility evaluator.
};

/** @brief Enumerates finite grid/yaw candidates without dropping page progress.
 */
class ViewpointSampler
{
public:
    /** @brief Samples a page from immutable geometry and requirement snapshots.
     * @param start Known current robot pose used by the reachability field.
     * @param map Consumer-owned geometry and coordinate epoch.
     * @param coverage Required work and measured quality ledger.
     * @param evaluator Sensor-specific visibility predictor.
     * @param regions Stable region assignment for accepted views.
     * @param active Sources supported by current sensing and localization.
     * @param safety Footprint and clearance policy; existing inflation is
     * respected.
     * @param options Candidate, examination and prediction limits.
     * @param sweep True enumerates every grid position without coarse stride.
     * @param cursor First slot in this page; cancellation preserves this
     * progress.
     * @param cancel_checker Cooperative cancellation, including field
     * preparation.
     * @param viewpoint_field Optional immutable measured 3-D clearance gate.
     * @throws std::invalid_argument For invalid options, domain size or stride.
     * The slot budget bounds enumeration only. Reachability-field construction
     * and each visibility prediction retain their own work and cancellation
     * cost.
     */
    ViewpointSamplingResult Sample(
        const Pose2d& start, const MapReadView& map,
        const CoverageState& coverage, const ViewpointEvaluator& evaluator,
        const RegionManager& regions, perception::SensorMask active,
        autonomy::navigation::SafetyOptions safety,
        ViewpointSamplingOptions options = {}, bool sweep = false,
        std::size_t cursor = 0,
        const std::function<bool()>& cancel_checker = {},
        std::shared_ptr<const RollingViewpointFieldView> viewpoint_field = {})
        const;
};
}  // namespace autonomy::task::exploration
