/* Copyright 2026 The Openbot Authors. */
/**
 * @file timed_bspline.hpp
 * @brief Immutable cubic reference with derivatives in physical seconds.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/common/math/bspline.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace sand_planner {

/** @brief XYZ in meters and an unwrapped yaw in radians; no implicit wrapping. */
using TrajectoryVector = std::array<double, 4>;

/** @brief Position and its first three physical-time derivatives. */
struct TrajectoryState {
    TrajectoryVector position{};      ///< XYZ/yaw, in meters/radians.
    TrajectoryVector velocity{};      ///< Meters/second and radians/second.
    TrajectoryVector acceleration{};  ///< Meters/second squared and radians/second squared.
    TrajectoryVector jerk{};          ///< Meters/second cubed and radians/second cubed.
    /** @brief Rejects nonfinite components in every derivative. */
    bool Finite() const;
};

/** @brief Execution identity, continuous frame and authoritative clock domain. */
struct TrajectoryTiming {
    std::string frame_id = "odom";  ///< Continuous execution frame; never an implicit map transform.
    std::string execution_id;        ///< Scope of the motion lease owning this reference.
    std::uint64_t alignment_epoch = 0;  ///< Frame alignment generation, supplied by its owner.
    std::uint64_t reference_version = 0;  ///< Monotonic replacement version within execution_id.
    double start_stamp = 0.0;        ///< Scheduled switch time in the execution clock, seconds.
    double duration = 0.0;           ///< Physical seconds corresponding to normalized u in [0,1].
    double valid_until = 0.0;        ///< Last authorized evaluation time, not renewed by retiming.
};

/** @brief Cubic control points and normalized, clamped, simple interior knots. */
struct BSplineData {
    std::vector<TrajectoryVector> control_points;  ///< Owned XYZ/yaw controls, never sampled waypoints.
    std::vector<double> knots;                    ///< Size is control_points.size()+4; endpoints repeat four times.
    TrajectoryTiming timing;                     ///< Physical-time and ownership metadata.
};

/** @brief Conservative derivative bounds over the entire represented curve. */
struct TrajectoryDerivativeBounds {
    double linear_velocity = 0.0;      ///< XYZ speed upper bound, meters/second.
    double linear_acceleration = 0.0;  ///< XYZ acceleration norm upper bound, meters/second squared.
    double linear_jerk = 0.0;          ///< XYZ jerk norm upper bound, meters/second cubed.
    double yaw_velocity = 0.0;         ///< Absolute yaw rate upper bound, radians/second.
    double yaw_acceleration = 0.0;     ///< Absolute yaw acceleration upper bound, radians/second squared.
    double yaw_jerk = 0.0;             ///< Absolute yaw jerk upper bound, radians/second cubed.
};

/**
 * @brief A bounded-size C2 cubic reference with explicit execution validity.
 *
 * Only simple interior knots are accepted. Jerk is piecewise constant and may
 * jump at interior knots. This class proves reference properties, not collision
 * freedom, nonholonomic feasibility, localization health or physical tracking.
 */
class TimedBSpline {
public:
    static constexpr std::size_t kMaximumControlPoints = 64;  ///< Hard input work bound.
    static constexpr double kMinimumKnotSpan = 1e-9;          ///< Minimum positive normalized span for numeric stability.

    /** @brief Validate and take ownership of a complete cubic reference.
     * @throws std::invalid_argument For malformed geometry, timing or identity.
     */
    explicit TimedBSpline(BSplineData data);
    /** @brief Immutable geometry and timing; callers must construct a new reference to change them. */
    const BSplineData& data() const { return data_; }
    /** @brief Last physical time represented and authorized by this reference. */
    double EndStamp() const;
    /** @brief Evaluate XYZ/yaw and derivatives, including right-sided jerk at interior knots.
     * @throws std::out_of_range Outside the represented and authorized interval.
     * @throws std::overflow_error If evaluation cannot represent finite derivatives.
     */
    TrajectoryState Evaluate(double stamp) const;
    /** @brief Derivative controls in physical units for orders 1, 2 or 3.
     * Their convex hull bounds the corresponding derivative on every span.
     * @throws std::invalid_argument For an unsupported derivative order.
     */
    std::vector<TrajectoryVector> DerivativeControlPoints(unsigned order) const;
    /** @brief Bound norms by derivative-control convex hulls; no temporal sampling. */
    TrajectoryDerivativeBounds DerivativeBounds() const;
    /** @brief Rebuild timing without silently reanchoring derivatives or extending validity. */
    TimedBSpline WithTiming(TrajectoryTiming timing) const;
    /** @brief Solve the first three controls from physical position, velocity and acceleration.
     * Uses the actual knot vector and duration. Far controls are preserved;
     * collision and derivative limits must be rechecked after this operation.
     * The caller supplies continuous, unwrapped yaw and the final switch state.
     */
    TimedBSpline WithStartState(const TrajectoryState& state) const;

private:
    BSplineData data_;  ///< Immutable after structural validation.
    std::array<::autonomy::common::math::ClampedCubicBSpline<double>, 4> curves_;  ///< Shared-math XYZ/yaw evaluators; no local spline algorithm.
};

}  // namespace sand_planner
}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
