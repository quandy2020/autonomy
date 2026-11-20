/* Copyright 2026 The Openbot Authors. */
/** @file viewpoint_sampler.cpp
 * @brief Bounded slot enumeration over immutable grid and yaw candidates.
 */
#include "autonomy/task/exploration/viewpoint/viewpoint_sampler.hpp"

#include <limits>
#include <stdexcept>
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"

namespace autonomy::task::exploration {
ViewpointSamplingResult ViewpointSampler::Sample(
    const Pose2d& start, const MapReadView& map, const CoverageState& state,
    const ViewpointEvaluator& evaluator, const RegionManager& regions,
    perception::SensorMask active, autonomy::navigation::SafetyOptions safety,
    ViewpointSamplingOptions options, bool sweep, std::size_t cursor,
    const std::function<bool()>& cancel_checker) const {
    if (!map.grid || !options.yaw_samples || options.yaw_samples > 360 ||
        !options.maximum_candidates || !options.maximum_evaluations || !options.maximum_examined ||
        !std::isfinite(options.spacing) || options.spacing <= 0 ||
        !std::isfinite(options.nominal_speed) || options.nominal_speed <= 0 ||
        !std::isfinite(options.angular_speed) || options.angular_speed <= 0 ||
        !std::isfinite(options.observation_time) || options.observation_time <= 0)
        throw std::invalid_argument("invalid viewpoint sampling configuration");
    if (map.startup_support) safety.startup_support = map.startup_support;
    const auto& grid = *map.grid;
    if (grid.size() > std::numeric_limits<std::size_t>::max() / options.yaw_samples)
        throw std::invalid_argument("viewpoint candidate domain exceeds cursor capacity");
    const std::size_t total = grid.size() * options.yaw_samples;
    const double requested_stride = sweep ? 1 : std::ceil(options.spacing / grid.resolution());
    if (!std::isfinite(requested_stride) || requested_stride > std::numeric_limits<int>::max())
        throw std::invalid_argument("viewpoint stride exceeds grid index capacity");
    const int stride = std::max(1, static_cast<int>(requested_stride));
    ViewpointSamplingResult result;
    result.next_cursor = std::min(cursor, total);
    if (cancel_checker && cancel_checker()) { result.canceled = true; return result; }
    if (cursor >= total) { result.finished = true; return result; }
    if (map.footprint_inflated) {
        safety.robot_radius = 1e-9;
        safety.horizontal_margin = 0;
    }
    auto geometry = std::make_shared<autonomy::navigation::GeometrySafety>(map.grid, safety);
    autonomy::navigation::ReachabilityField reachable(geometry, start, 0, cancel_checker);
    if (reachable.canceled()) {
        result.canceled = true;
        return result;
    }
    const auto initial_cursor = cursor;
    for (; cursor < total; ++cursor) {
        if (cancel_checker && cancel_checker()) {
            result.canceled = true;
            result.next_cursor = cursor;
            return result;
        }
        if (result.examined >= options.maximum_examined ||
            result.evaluated >= options.maximum_evaluations ||
            result.candidates.size() >= options.maximum_candidates) {
            result.limited = true;
            result.next_cursor = cursor;
            return result;
        }
        ++result.examined;
        result.next_cursor = cursor + 1;
        const auto cell = grid.FromIndex(cursor / options.yaw_samples);
        if (cell.x % stride || cell.y % stride ||
            grid.At(cell.x, cell.y).state != autonomy::navigation::CellState::kFree) continue;
        auto pose = grid.CellCenter(cell.x, cell.y);
        const double distance = reachable.DistanceTo(pose);
        if (!std::isfinite(distance)) continue;
        pose.yaw = 2 * autonomy::navigation::kPi * (cursor % options.yaw_samples) /
                   options.yaw_samples;
        auto contribution = evaluator.Predict(pose, state, map, active);
        const double gain = CoverageFusion::Gain(state, contribution);
        ++result.evaluated;
        if (gain > 0) {
            Viewpoint view;
            view.id = cursor + 1;
            view.pose = pose;
            view.region_id = regions.RegionFor(pose);
            view.contributions = std::move(contribution);
            view.predicted_gain = gain;
            view.travel_time = distance / options.nominal_speed;
            view.turn_time = std::abs(autonomy::navigation::NormalizeAngle(pose.yaw - start.yaw)) /
                             options.angular_speed;
            view.observation_time = options.observation_time;
            result.candidates.push_back(std::move(view));
        }
    }
    result.finished = true;
    result.exhaustive = sweep && initial_cursor == 0;
    return result;
}
}  // namespace autonomy::task::exploration
