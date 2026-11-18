/* Copyright 2026 The Openbot Authors. */
/** @file local_trajectory_safety.cpp
 * @brief Concrete full-body sweep admission using immutable measured voxel geometry.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/local_trajectory_safety.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/local_ground.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief Positive finite configuration values keep all work and stopping bounds defined. */
bool Positive(double x) { return std::isfinite(x) && x > 0.0; }
/** @brief Explicit XYZ access, without layout assumptions. */
std::array<double, 3> Components(const perception::Vec3& p) { return {p.x, p.y, p.z}; }

/** @brief Aggregate clock/work scope shared by all adaptive chords. */
class SweepBudget {
public:
    /** @brief All backend work shares the caller's one deadline. */
    SweepBudget(std::size_t maximum, double deadline, const ExecutionClock& clock, double initial)
        : maximum_(maximum), deadline_(deadline), clock_(clock), last_(initial) {}
    /** @brief Reject work before increment and poll at least every 64 operations. */
    void Spend() {
        if (used_ == maximum_) throw std::runtime_error("local trajectory whole-body budget exhausted");
        ++used_;
        if ((used_ & 63U) == 0) Check();
    }
    /** @brief Fail closed on time regression or expiration. */
    double Check() {
        const double now = clock_();
        if (!std::isfinite(now) || now < last_ || now >= deadline_)
            throw std::runtime_error("local trajectory geometry clock regressed or deadline expired");
        last_ = now;
        return now;
    }
    /** @brief Remaining work shared with the concrete measured-ground consumer. */
    std::size_t remaining() const { return maximum_ - used_; }
private:
    std::size_t maximum_;  ///< Entire trajectory work bound.
    double deadline_;      ///< Exclusive execution-clock deadline.
    const ExecutionClock& clock_;  ///< Caller-owned clock for this synchronous check.
    double last_;          ///< Last checked clock value.
    std::size_t used_ = 0;  ///< Samples, cells and ground queries share this counter.
};

/** @brief Check a complete expanded cylinder against all intersecting voxel volumes.
 * Exact cell/AABB intersections are conservative at boundaries. An ESDF sphere
 * shortcut is used only when its lower bound encloses the entire cylinder.
 */
GeometryStatus CheckCylinder(const LocalEsdfSnapshot& snapshot, const perception::Vec3& center,
                            const RobotBodyEnvelope& body, double now,
                            SweepBudget& budget, LocalTrajectorySafetyResult& result) {
    const double middle_z = (body.minimum_z + body.maximum_z) / 2.0;
    const double sphere = std::hypot(body.horizontal_radius, (body.maximum_z - body.minimum_z) / 2.0);
    if (snapshot.DistanceLowerBound(center + perception::Vec3{0, 0, middle_z}, now) > sphere)
        return GeometryStatus::kClear;
    const auto& o = snapshot.options();
    const auto origin = Components(o.origin), p = Components(center);
    const std::array<double, 3> lo{p[0] - body.horizontal_radius, p[1] - body.horizontal_radius,
                                 p[2] + body.minimum_z};
    const std::array<double, 3> hi{p[0] + body.horizontal_radius, p[1] + body.horizontal_radius,
                                 p[2] + body.maximum_z};
    std::array<unsigned, 3> begin{}, end{};
    for (unsigned a = 0; a < 3; ++a) {
        const double maximum = origin[a] + o.dimensions[a] * o.resolution;
        if (!std::isfinite(lo[a]) || !std::isfinite(hi[a]) || lo[a] <= origin[a] || hi[a] >= maximum)
            return GeometryStatus::kIndeterminate;
        begin[a] = static_cast<unsigned>(std::floor(std::nextafter(
            (lo[a] - origin[a]) / o.resolution, -std::numeric_limits<double>::infinity())));
        end[a] = static_cast<unsigned>(std::floor((hi[a] - origin[a]) / o.resolution));
    }
    for (unsigned z = begin[2]; z <= end[2]; ++z)
        for (unsigned y = begin[1]; y <= end[1]; ++y)
            for (unsigned x = begin[0]; x <= end[0]; ++x) {
                budget.Spend();
                ++result.examined_voxels;
                const double lower_x = origin[0] + x * o.resolution;
                const double lower_y = origin[1] + y * o.resolution;
                const double dx = std::max({lower_x - p[0], p[0] - lower_x - o.resolution, 0.0});
                const double dy = std::max({lower_y - p[1], p[1] - lower_y - o.resolution, 0.0});
                if (dx * dx + dy * dy > body.horizontal_radius * body.horizontal_radius) continue;
                const auto state = snapshot.State(x, y, z, now);
                if (state == LocalVoxelState::kOccupied) return GeometryStatus::kBlocked;
                if (state != LocalVoxelState::kFree) return GeometryStatus::kIndeterminate;
            }
    return GeometryStatus::kClear;
}
}  // namespace

LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options, const LocalTrajectorySafetyOptions& s,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const GroundSupportVerifier& ground_support, const ExecutionClock& clock) {
    LocalTrajectorySafetyResult result;
    const auto& timing = trajectory.data().timing;
    if (!snapshot || !clock || !ground_support || !Positive(s.body.horizontal_radius) ||
        !std::isfinite(s.body.minimum_z) || !std::isfinite(s.body.maximum_z) ||
        s.body.minimum_z >= s.body.maximum_z || !std::isfinite(s.position_margin) || s.position_margin < 0.0 ||
        !std::isfinite(s.reaction_time) || s.reaction_time < 0.0 ||
        !Positive(s.minimum_braking_deceleration) || !Positive(s.maximum_sample_spacing) ||
        !s.maximum_work || s.maximum_work > 100000000 || s.clock_domain.empty() ||
        s.clock_domain != snapshot->options().clock_domain || timing.frame_id != snapshot->options().execution_frame ||
        timing.alignment_epoch != snapshot->alignment_epoch() ||
        validation_options.geometry_revision != snapshot->options().revision) {
        result.validation.reason = "missing ground evidence, invalid body/budget or mismatched snapshot identity";
        return result;
    }
    try {
        const double initial = clock();
        if (!std::isfinite(initial) || initial < snapshot->built_at() || initial > snapshot->valid_until() ||
            timing.start_stamp < snapshot->built_at() || trajectory.EndStamp() > snapshot->valid_until()) {
            result.validation.reason = "local trajectory snapshot expired or does not cover execution";
            return result;
        }
        const double speed = trajectory.DerivativeBounds().linear_velocity;
        const double stopping = speed * s.reaction_time + speed * speed / (2.0 * s.minimum_braking_deceleration);
        if (!std::isfinite(stopping)) throw std::invalid_argument("stopping distance overflow");
        SweepBudget budget(s.maximum_work, validation_options.deadline, clock, initial);
        const auto verifier = [&](const SweptVolumeQuery& q) {
            GeometryVerdict verdict;
            verdict.geometry_revision = snapshot->options().revision;
            verdict.valid_until = snapshot->valid_until();
            const double now = budget.Check();
            if (q.end_stamp > snapshot->valid_until() || now > snapshot->valid_until()) return verdict;
            const perception::Vec3 begin{q.begin.position[0], q.begin.position[1], q.begin.position[2]};
            const perception::Vec3 end{q.end.position[0], q.end.position[1], q.end.position[2]};
            const double length = (end - begin).Norm();
            const double sample_count = std::max(1.0, std::ceil(length / s.maximum_sample_spacing));
            if (!std::isfinite(sample_count) || sample_count > s.maximum_work)
                throw std::runtime_error("local trajectory chord exceeds whole-body work bound");
            const auto count = static_cast<std::size_t>(sample_count);
            // Each midpoint covers its entire chord subinterval. The same
            // Euclidean translation margin expands horizontal radius and Z.
            const double margin = length / (2.0 * count) + q.linear_deviation + s.position_margin +
                                  snapshot->alignment_error() + snapshot->options().measurement_error;
            RobotBodyEnvelope expanded{s.body.horizontal_radius + margin + stopping,
                                       s.body.minimum_z - margin, s.body.maximum_z + margin};
            for (std::size_t i = 0; i < count; ++i) {
                budget.Spend();
                ++result.sweep_samples;
                const auto center = begin + (end - begin) * ((i + 0.5) / count);
                verdict.status = CheckCylinder(*snapshot, center, expanded, now, budget, result);
                if (verdict.status != GeometryStatus::kClear) return verdict;
            }
            budget.Spend();
            ++result.support_queries;
            const auto support = ground_support(GroundSupportQuery{q, expanded, stopping, budget.remaining(),
                                                                   s.position_margin});
            if (support.work > budget.remaining())
                throw std::runtime_error("ground consumer exceeds remaining whole-trajectory work");
            result.ground_work += support.work;
            for (std::size_t work = 0; work < support.work; ++work) budget.Spend();
            budget.Check();
            if (support.status != GeometryStatus::kClear || support.frame_id != timing.frame_id ||
                support.alignment_epoch != timing.alignment_epoch || support.geometry_revision != q.geometry_revision ||
                !std::isfinite(support.valid_until) || support.valid_until < q.end_stamp) {
                verdict.status = GeometryStatus::kIndeterminate;
                return verdict;
            }
            verdict.status = GeometryStatus::kClear;
            verdict.valid_until = std::min(verdict.valid_until, support.valid_until);
            return verdict;
        };
        result.validation = ValidateTrajectory(trajectory, limits, validation_options, verifier, clock);
    } catch (const std::exception& exception) {
        result.validation.reason = exception.what();
    } catch (...) {
        result.validation.reason = "nonstandard local trajectory safety dependency exception";
    }
    return result;
}

LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options,
    const LocalTrajectorySafetyOptions& safety_options,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const std::shared_ptr<const LocalGroundSnapshot>& ground, const ExecutionClock& clock) {
    if (!ground || !snapshot || ground->geometry().get() != snapshot.get()) {
        LocalTrajectorySafetyResult result;
        result.validation.reason = "measured ground does not pin the exact ESDF instance";
        return result;
    }
    const GroundSupportVerifier consumer = [&](const GroundSupportQuery& query) {
        return VerifyGroundSupport(*ground, query, clock);
    };
    return ValidateLocalTrajectory(trajectory, limits, validation_options, safety_options,
                                   snapshot, consumer, clock);
}

LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options,
    const LocalTrajectorySafetyOptions& safety_options,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const std::shared_ptr<const LocalGroundSnapshot>& ground,
    const DifferentialDriveLimits& drive, const ExecutionClock& clock) {
    LocalTrajectorySafetyResult result;
    if (!clock || !safety_options.maximum_work || safety_options.maximum_work > 100000000 ||
        !ground || !snapshot || ground->geometry().get() != snapshot.get()) {
        result.validation.reason = "missing drive execution clock, work budget or exact measured ground instance";
        return result;
    }
    try {
        double last_clock = -1.0;
        // Keep clock continuity across the model proof and concrete geometry
        // stages, including each nested ground and voxel consumer.
        const ExecutionClock shared_clock = [&] {
            const double now = clock();
            if (!std::isfinite(now) || now < 0.0 || now < last_clock)
                throw std::runtime_error("drive/body/ground shared clock regressed or is invalid");
            last_clock = now;
            return now;
        };
        auto model_options = validation_options;
        model_options.maximum_segments = static_cast<unsigned>(std::min<std::size_t>(
            model_options.maximum_segments, safety_options.maximum_work));
        result.differential_drive = ValidateDifferentialDrive(trajectory, drive, model_options, shared_clock);
        if (!result.differential_drive.admitted) {
            result.validation.reason = result.differential_drive.reason;
            return result;
        }
        const auto used = result.differential_drive.examined_intervals;
        if (used >= validation_options.maximum_segments || used >= safety_options.maximum_work) {
            result.validation.reason = "drive proof leaves no shared budget for body and ground validation";
            return result;
        }
        auto remaining_validation = validation_options;
        remaining_validation.maximum_segments -= used;
        auto remaining_safety = safety_options;
        remaining_safety.maximum_work -= used;
        auto body_result = ValidateLocalTrajectory(trajectory, limits, remaining_validation, remaining_safety,
                                                  snapshot, ground, shared_clock);
        body_result.differential_drive = std::move(result.differential_drive);
        return body_result;
    } catch (const std::exception& error) {
        result.validation.reason = error.what();
    } catch (...) {
        result.validation.reason = "nonstandard drive/body/ground shared validation exception";
    }
    return result;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
