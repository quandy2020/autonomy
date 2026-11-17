/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/viewpoint/viewpoint_sampler.hpp"

#include <stdexcept>
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"

namespace autonomy::task::exploration {
ViewpointSamplingResult ViewpointSampler::Sample(
    const Pose2d& start, const MapReadView& map, const CoverageState& state,
    const ViewpointEvaluator& evaluator, const RegionManager& regions,
    perception::SensorMask active, opt::navigation::SafetyOptions safety,
    ViewpointSamplingOptions options, bool sweep, std::size_t cursor,
    const std::function<bool()>& cancel_checker) const {
    if (!map.grid || !options.yaw_samples || options.yaw_samples > 360 ||
        !options.maximum_candidates || !options.maximum_evaluations ||
        !std::isfinite(options.spacing) || options.spacing <= 0 ||
        !std::isfinite(options.nominal_speed) || options.nominal_speed <= 0 ||
        !std::isfinite(options.angular_speed) || options.angular_speed <= 0 ||
        !std::isfinite(options.observation_time) || options.observation_time <= 0)
        throw std::invalid_argument("invalid viewpoint sampling configuration");
    if (map.footprint_inflated) {
        safety.robot_radius = 1e-9;
        safety.horizontal_margin = 0;
    }
    ViewpointSamplingResult result;
    auto geometry = std::make_shared<opt::navigation::GeometrySafety>(map.grid, safety);
    opt::navigation::ReachabilityField reachable(geometry, start, 0, cancel_checker);
    if (reachable.canceled()) {
        result.canceled = true;
        return result;
    }
    const auto& grid = *map.grid;
    const int stride = sweep ? 1 : std::max(1, static_cast<int>(
        std::ceil(options.spacing / grid.resolution())));
    const std::size_t total = grid.size() * options.yaw_samples;
    const auto initial_cursor = cursor;
    std::size_t evaluated = 0;
    for (; cursor < total; ++cursor) {
        if (cancel_checker && cancel_checker()) {
            result.canceled = true;
            result.next_cursor = cursor;
            return result;
        }
        const auto cell = grid.FromIndex(cursor / options.yaw_samples);
        if (cell.x % stride || cell.y % stride ||
            grid.At(cell.x, cell.y).state != opt::navigation::CellState::kFree) continue;
        auto pose = grid.CellCenter(cell.x, cell.y);
        const double distance = reachable.DistanceTo(pose);
        if (!std::isfinite(distance)) continue;
        pose.yaw = 2 * opt::navigation::kPi * (cursor % options.yaw_samples) /
                   options.yaw_samples;
        auto contribution = evaluator.Predict(pose, state, map, active);
        const double gain = CoverageFusion::Gain(state, contribution);
        ++evaluated;
        if (gain > 0) {
            Viewpoint view;
            view.id = cursor + 1;
            view.pose = pose;
            view.region_id = regions.RegionFor(pose);
            view.contributions = std::move(contribution);
            view.predicted_gain = gain;
            view.travel_time = distance / options.nominal_speed;
            view.turn_time = std::abs(opt::navigation::NormalizeAngle(pose.yaw - start.yaw)) /
                             options.angular_speed;
            view.observation_time = options.observation_time;
            result.candidates.push_back(std::move(view));
        }
        if (result.candidates.size() >= options.maximum_candidates ||
            evaluated >= options.maximum_evaluations) {
            result.next_cursor = cursor + 1;
            result.finished = result.next_cursor == total;
            result.limited = !result.finished;
            result.exhaustive = sweep && initial_cursor == 0 && result.finished;
            return result;
        }
    }
    result.finished = true;
    result.exhaustive = sweep && initial_cursor == 0;
    return result;
}
}  // namespace autonomy::task::exploration
