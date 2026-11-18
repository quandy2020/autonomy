/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_validator.cpp
 * @brief Adaptive swept intervals with pinned revision, freshness and work limits.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief Explicitly positive finite options prevent division and endless subdivision. */
bool Positive(double x) { return std::isfinite(x) && x > 0.0; }
/** @brief An interval awaiting conservative envelope checks. */
struct Interval {
    double begin;  ///< Inclusive execution-clock start.
    double end;    ///< Inclusive execution-clock end.
};
}  // namespace

TrajectoryValidationResult ValidateTrajectory(const TimedBSpline& trajectory,
                                             const TrajectoryLimits& limits,
                                             const TrajectoryValidationOptions& o,
                                             const SweptVolumeVerifier& verifier,
                                             const ExecutionClock& clock) {
    TrajectoryValidationResult result;
    if (!limits.Valid() || !verifier || !clock || !Positive(o.maximum_segment_duration) ||
        !Positive(o.maximum_linear_deviation) || !Positive(o.maximum_yaw_deviation) ||
        !o.maximum_segments || o.maximum_segments > 65536 || !o.geometry_revision || !Positive(o.deadline)) {
        result.reason = "invalid validation limits, callbacks, geometry revision or budget";
        return result;
    }
    try {
        const auto& timing = trajectory.data().timing;
        const double end = timing.start_stamp + timing.duration;
        double last_clock = clock();
        if (!std::isfinite(last_clock) || last_clock < 0.0 || last_clock >= o.deadline ||
            last_clock > timing.start_stamp || end > timing.valid_until) {
            result.reason = "late reference, invalid clock, expired deadline or truncated execution horizon";
            return result;
        }
        const auto bounds = trajectory.DerivativeBounds();
        if (!limits.Contains(bounds)) {
            result.reason = "final reference violates derivative hull limits";
            return result;
        }
        std::vector<Interval> pending{{timing.start_stamp, end}};
        double evidence_until = timing.valid_until;
        while (!pending.empty()) {
            const double now = clock();
            if (!std::isfinite(now) || now < last_clock || now >= o.deadline || now > timing.start_stamp) {
                result.reason = "validation clock regressed, timed out or missed the scheduled start";
                return result;
            }
            last_clock = now;
            if (result.examined_segments == o.maximum_segments) {
                result.reason = "adaptive swept validation exhausted its work budget";
                return result;
            }
            ++result.examined_segments;
            const auto interval = pending.back();
            pending.pop_back();
            const double h = interval.end - interval.begin;
            const double linear = bounds.linear_acceleration * h * h / 8.0;
            const double angular = bounds.yaw_acceleration * h * h / 8.0;
            if (!std::isfinite(linear) || !std::isfinite(angular)) {
                result.reason = "swept chord envelope overflow";
                return result;
            }
            if (h > o.maximum_segment_duration || linear > o.maximum_linear_deviation ||
                angular > o.maximum_yaw_deviation) {
                const double middle = interval.begin + h / 2.0;
                if (middle <= interval.begin || middle >= interval.end) {
                    result.reason = "execution-clock precision cannot represent a safe subdivision";
                    return result;
                }
                pending.push_back({middle, interval.end});
                pending.push_back({interval.begin, middle});
                continue;
            }
            SweptVolumeQuery query;
            query.begin = trajectory.Evaluate(interval.begin);
            query.end = trajectory.Evaluate(interval.end);
            query.begin_stamp = interval.begin;
            query.end_stamp = interval.end;
            query.linear_deviation = linear;
            query.yaw_deviation = angular;
            query.geometry_revision = o.geometry_revision;
            query.timing = timing;
            query.deadline = o.deadline;
            const auto verdict = verifier(query);
            if (verdict.status != GeometryStatus::kClear || verdict.geometry_revision != o.geometry_revision ||
                !std::isfinite(verdict.valid_until) || verdict.valid_until < interval.end) {
                result.reason = "swept geometry is blocked, unknown, expired or from a different revision";
                return result;
            }
            ++result.verified_segments;
            evidence_until = std::min(evidence_until, verdict.valid_until);
        }
        const double finished = clock();
        if (!std::isfinite(finished) || finished < last_clock || finished >= o.deadline ||
            finished > timing.start_stamp || finished > evidence_until) {
            result.reason = "validation completed too late or with an invalid clock";
            return result;
        }
        result.admitted = true;
        result.valid_until = evidence_until;
    } catch (const std::exception& exception) {
        result.reason = exception.what();
    } catch (...) {
        result.reason = "nonstandard exception from independent validation backend";
    }
    return result;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
