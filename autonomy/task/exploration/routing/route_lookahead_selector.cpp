/* Copyright 2026 The Openbot Authors. */

#include "autonomy/task/exploration/routing/route_lookahead_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autonomy::task::exploration {
namespace {

constexpr double kPoseTolerance = 1e-6;

bool SamePose(const autonomy::navigation::Pose2d& lhs,
              const autonomy::navigation::Pose2d& rhs) {
    return autonomy::navigation::Distance(lhs, rhs) <= kPoseTolerance &&
           std::abs(autonomy::navigation::NormalizeAngle(lhs.yaw - rhs.yaw)) <=
               kPoseTolerance;
}

}  // namespace

RouteLookaheadResult RouteLookaheadSelector::Select(
    const planning::planner::vista::VistaRoute& route,
    const autonomy::navigation::Pose2d& start,
    const autonomy::navigation::Pose2d& goal, double lookahead_distance,
    std::uint64_t expected_epoch, std::uint64_t expected_geometry_revision,
    std::string_view expected_frame, std::uint64_t expected_field_generation,
    const RouteLineOfSightView* line_of_sight,
    const std::function<bool()>& canceled) const {
    RouteLookaheadResult result;
    if (canceled && canceled()) {
        result.status = RouteLookaheadStatus::kCanceled;
        return result;
    }
    if (!route.success || route.canceled || route.attempted ||
        route.execution_verified || !autonomy::navigation::IsFinite(start) ||
        !autonomy::navigation::IsFinite(goal) ||
        !std::isfinite(lookahead_distance) || lookahead_distance <= 0 ||
        !std::isfinite(route.length) || route.length < 0 ||
        expected_epoch == 0 || expected_geometry_revision == 0 ||
        expected_frame.empty() || route.path.poses.empty()) {
        return result;
    }
    if (!route.evidence) {
        result.status = RouteLookaheadStatus::kMissingEvidence;
        return result;
    }
    if (route.map_epoch != expected_epoch ||
        route.geometry_revision != expected_geometry_revision ||
        route.evidence->map_epoch != expected_epoch ||
        route.evidence->geometry_revision != expected_geometry_revision ||
        std::string_view(route.path.frame_id) != expected_frame ||
        std::string_view(route.evidence->frame_id) != expected_frame) {
        result.status = RouteLookaheadStatus::kStaleRoute;
        return result;
    }
    if (line_of_sight &&
        (expected_field_generation == 0 || !line_of_sight->check ||
         line_of_sight->map_epoch != expected_epoch ||
         line_of_sight->geometry_revision != expected_geometry_revision ||
         line_of_sight->field_generation != expected_field_generation ||
         std::string_view(line_of_sight->frame_id) != expected_frame)) {
        result.status = RouteLookaheadStatus::kLineOfSightUnavailable;
        return result;
    }

    const auto& poses = route.path.poses;
    if (!SamePose(poses.front(), start) || !SamePose(poses.back(), goal)) {
        return result;
    }
    std::vector<double> cumulative(poses.size(), 0.0);
    for (std::size_t index = 0; index < poses.size(); ++index) {
        if (canceled && canceled()) {
            result.status = RouteLookaheadStatus::kCanceled;
            return result;
        }
        if (!autonomy::navigation::IsFinite(poses[index])) {
            return result;
        }
        if (index > 0) {
            const double segment =
                autonomy::navigation::Distance(poses[index - 1], poses[index]);
            if (!std::isfinite(segment) ||
                segment > std::numeric_limits<double>::max() -
                              cumulative[index - 1]) {
                return result;
            }
            cumulative[index] = cumulative[index - 1] + segment;
            if (!std::isfinite(cumulative[index])) {
                return result;
            }
        }
    }
    const double total = cumulative.back();
    if (!std::isfinite(total)) {
        return result;
    }
    const double length_tolerance = 1e-8 * std::max({1.0, total, route.length});
    if (!std::isfinite(length_tolerance) ||
        std::abs(total - route.length) > length_tolerance) {
        return result;
    }

    const double requested_arc = std::min(lookahead_distance, total);
    std::optional<std::size_t> forced_visible_index;
    if (line_of_sight) {
        std::size_t required_index = poses.size() - 1;
        if (requested_arc + length_tolerance < total) {
            required_index = 1;
            while (required_index + 1 < poses.size() &&
                   cumulative[required_index] + length_tolerance <
                       requested_arc) {
                ++required_index;
            }
        }
        for (std::size_t index = 1; index <= required_index; ++index) {
            if (canceled && canceled()) {
                result.status = RouteLookaheadStatus::kCanceled;
                return result;
            }
            RouteLineOfSightStatus status =
                RouteLineOfSightStatus::kUnavailable;
            try {
                status = line_of_sight->check(start, poses[index]);
            } catch (...) {
                result.status = RouteLookaheadStatus::kLineOfSightUnavailable;
                return result;
            }
            if (status == RouteLineOfSightStatus::kCanceled) {
                result.status = RouteLookaheadStatus::kCanceled;
                return result;
            }
            if (status == RouteLineOfSightStatus::kUnavailable) {
                result.status = RouteLookaheadStatus::kLineOfSightUnavailable;
                return result;
            }
            if (status == RouteLineOfSightStatus::kBlocked) {
                forced_visible_index = index - 1;
                result.line_of_sight_limited = true;
                break;
            }
        }
    }

    double selected_arc = requested_arc;
    if (forced_visible_index) {
        selected_arc = cumulative[*forced_visible_index];
        result.pose = poses[*forced_visible_index];
        if (*forced_visible_index > 0) {
            const auto& previous = poses[*forced_visible_index - 1];
            result.pose.yaw = std::atan2(result.pose.y - previous.y,
                                         result.pose.x - previous.x);
        }
    } else if (poses.size() == 1 || selected_arc <= length_tolerance) {
        result.pose = poses.front();
    } else if (std::abs(selected_arc - total) <= length_tolerance) {
        result.pose = poses.back();
    } else {
        std::size_t upper = 1;
        while (upper + 1 < poses.size() &&
               cumulative[upper] + length_tolerance < selected_arc) {
            ++upper;
        }
        if (std::abs(cumulative[upper] - selected_arc) <= length_tolerance) {
            result.pose = poses[upper];
            std::size_t outgoing = upper + 1;
            while (outgoing < poses.size() &&
                   cumulative[outgoing] - cumulative[upper] <=
                       length_tolerance) {
                ++outgoing;
            }
            if (outgoing < poses.size()) {
                result.pose.yaw = std::atan2(poses[outgoing].y - result.pose.y,
                                             poses[outgoing].x - result.pose.x);
            }
        } else {
            const auto& from = poses[upper - 1];
            const auto& to = poses[upper];
            const double segment = cumulative[upper] - cumulative[upper - 1];
            if (segment <= length_tolerance) {
                return result;
            }
            const double ratio = std::clamp(
                (selected_arc - cumulative[upper - 1]) / segment, 0.0, 1.0);
            result.pose.x = from.x + ratio * (to.x - from.x);
            result.pose.y = from.y + ratio * (to.y - from.y);
            result.pose.yaw = std::atan2(to.y - from.y, to.x - from.x);
        }
    }

    result.route_arc_length = selected_arc;
    result.final_endpoint = !result.line_of_sight_limited &&
                            std::abs(selected_arc - total) <= length_tolerance;
    if (result.final_endpoint) {
        result.pose = poses.back();
    }
    if (canceled && canceled()) {
        result = {};
        result.status = RouteLookaheadStatus::kCanceled;
        return result;
    }
    result.status = RouteLookaheadStatus::kSelected;
    return result;
}

}  // namespace autonomy::task::exploration
