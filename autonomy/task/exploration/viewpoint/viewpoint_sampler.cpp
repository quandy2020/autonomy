/* Copyright 2026 The Openbot Authors. */
/** @file viewpoint_sampler.cpp
 * @brief Bounded slot enumeration over immutable grid and yaw candidates.
 */
#include "autonomy/task/exploration/viewpoint/viewpoint_sampler.hpp"

#include <limits>
#include <stdexcept>
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"
#include "autonomy/common/logging.hpp"

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
        !std::isfinite(options.observation_time) || options.observation_time <= 0 ||
        !std::isfinite(options.goal_clearance_reserve) ||
        options.goal_clearance_reserve < 0 || options.goal_clearance_reserve > .30)
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
    const auto start_check = geometry->VerifyPose(start);
    AINFO_EVERY(20) << "EXPLORATION_SAMPLER start=(" << start.x << ","
                    << start.y << ") safe=" << start_check.safe
                    << " reason=" << start_check.reason
                    << " support=" << (map.startup_support ? 1 : 0)
                    << " grid=" << grid.width() << "x" << grid.height()
                    << " stride=" << stride << " cursor=" << cursor;
    // The reachability field is empty unless its origin is verifiably safe. A robot
    // standing on a cell the mapper has not certified as free would then make every
    // viewpoint unreachable, so fall back to the nearest safe neighbouring cell.
    Pose2d origin = start;
    if (!start_check.safe) {
        const auto cell = grid.WorldToCell(start);
        if (cell) {
            const int span = std::max(1, static_cast<int>(std::ceil(.60 / grid.resolution())));
            double best = std::numeric_limits<double>::infinity();
            for (int dy = -span; dy <= span; ++dy) for (int dx = -span; dx <= span; ++dx) {
                const int x = cell->x + dx, y = cell->y + dy;
                if (!grid.Contains(x, y) || grid.At(x, y).state != autonomy::navigation::CellState::kFree)
                    continue;
                auto candidate = grid.CellCenter(x, y);
                candidate.yaw = start.yaw;
                const double distance = autonomy::navigation::Distance(start, candidate);
                if (distance <= 1e-9 || distance > .60 || distance >= best) continue;
                if (!geometry->VerifyPose(candidate, options.goal_clearance_reserve).safe)
                    continue;
                best = distance; origin = candidate;
            }
            if (best < std::numeric_limits<double>::infinity()) {
                AINFO_EVERY(20) << "EXPLORATION_SAMPLER start_anchor=(" << origin.x << ","
                                << origin.y << ") distance=" << best;
            }
        }
    }
    autonomy::navigation::ReachabilityField reachable(geometry, origin, 0, cancel_checker);
    if (reachable.canceled()) {
        result.canceled = true;
        return result;
    }
    // If the measured pose is inside the near-wall admission band, the
    // bounded safe anchor is useful as a recovery viewpoint itself, not only
    // as the seed of a reachability field. Without this candidate the planner
    // can see many useful frontier cells but no route from an unsafe start.
    if (!start_check.safe && autonomy::navigation::Distance(origin, start) > 1e-9) {
        origin.yaw = start.yaw;
        if (geometry->VerifyPose(origin).safe) {
            const double distance = autonomy::navigation::Distance(start, origin);
            for (unsigned sample = 0; sample < options.yaw_samples &&
                 result.candidates.size() < options.maximum_candidates; ++sample) {
                Pose2d view_pose = origin;
                view_pose.yaw = 2 * autonomy::navigation::kPi * sample / options.yaw_samples;
                auto contribution = evaluator.Predict(view_pose, state, map, active);
                const double gain = CoverageFusion::Gain(state, contribution);
                if (gain <= 0) continue;
                ++result.evaluated;
                Viewpoint view;
                view.id = total + sample + 1;
                view.pose = view_pose;
                view.region_id = regions.RegionFor(view_pose);
                view.contributions = std::move(contribution);
                view.predicted_gain = gain;
                view.travel_time = distance / options.nominal_speed;
                view.turn_time = std::abs(autonomy::navigation::NormalizeAngle(
                    view_pose.yaw - start.yaw)) / options.angular_speed;
                view.observation_time = options.observation_time;
                result.candidates.push_back(std::move(view));
            }
        }
    }
    int checked_cell = -1;
    bool cell_safe = false;
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
        // The route owner admits only poses its conservative footprint check
        // certifies. Proposing a viewpoint the router must reject left the
        // session with candidates but no reachable local route.
        const int cell_index = static_cast<int>(cursor / options.yaw_samples);
        if (cell_index != checked_cell) {
            checked_cell = cell_index;
            // A viewpoint is also an execution target. Keep a small clearance
            // reserve beyond the robot radius so ordinary tracking error and a
            // newly observed neighbouring cell do not turn a nominally safe goal
            // into an unrecoverable command-gate rejection on approach.
            const auto verified = geometry->VerifyPose(pose, options.goal_clearance_reserve);
            // Vista may admit a bounded escape for a start pose, but its execution
            // planner requires every goal to pass the footprint check.
            cell_safe = verified.safe;
        }
        if (!cell_safe) continue;
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
