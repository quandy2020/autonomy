/* Copyright 2026 The Openbot Authors. */
/** @file local_ground.hpp
 * @brief Immutable measured support patches and whole stopping-footprint checks.
 */
#pragma once

#include <memory>
#include <vector>

#include "autonomy/control/controller/policy_controller/sand_planner/local_trajectory_safety.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Explicit owner attestation; frame names do not imply gravity alignment.
 * A visual/LiDAR localization installation may provide this contract without an
 * IMU. The calibrated reference height constrains a measured patch; it never
 * supplies floor samples or fills missing depth. Material strength is separate.
 */
struct GroundCalibration {
    std::string execution_frame;          ///< Exact continuous frame of measured patches.
    std::string clock_domain;             ///< Same clock as acquisition and execution.
    std::uint64_t alignment_epoch = 0;    ///< Owner's continuous-frame generation.
    double valid_from = 0.0;              ///< Start of the owner attestation interval.
    double valid_until = 0.0;             ///< End of the owner attestation interval.
    double reference_height = 0.0;        ///< Reference XYZ height above supporting floor, meters.
    double height_error = 0.01;           ///< Calibrated reference-to-floor error, meters.
    double gravity_angle_error = 0.0;     ///< Conservative vertical-axis uncertainty, radians.
    bool gravity_aligned = false;         ///< Explicit installation/localization-owner statement.
    bool healthy = false;                 ///< Owner health, independently required.
};

/** @brief Fixed sensing resolution, robot support limits and bounded construction. */
struct LocalGroundOptions {
    GroundCalibration calibration;       ///< Required calibrated and time-valid reference contract.
    unsigned minimum_samples = 4;        ///< Valid samples and all four XY quadrants are required.
    double minimum_quality = 0.2;         ///< Minimum actual geometric sample quality.
    double minimum_covariance = 0.005;    ///< Least XY eigenvalue divided by squared cell resolution.
    double maximum_residual = 0.02;       ///< Largest permitted measured plane residual, meters.
    double maximum_slope = 0.25;          ///< Calibrated admissible slope including axis error, radians.
    double maximum_reference_error = 0.08; ///< Allowed measured-floor/reference discrepancy, meters.
    double maximum_step = 0.05;           ///< Maximum upward neighbor discontinuity, meters.
    double maximum_drop = 0.05;           ///< Maximum downward neighbor discontinuity, meters.
    std::size_t maximum_work = 2000000;   ///< Build work, at most 100 million operations.
    double deadline = 0.0;                ///< Exclusive construction deadline in the shared clock.
};

/** @brief Measured planar support at one ESDF XY column; default has no support. */
struct GroundPatch {
    bool supported = false;             ///< Complete quadrants, stable fit and bounded slope/residual.
    double center_height = 0.0;         ///< Fitted measured Z at this column's center, meters.
    double slope_x = 0.0, slope_y = 0.0; ///< Measured gradients dZ/dX and dZ/dY.
    double height_error = 0.0;          ///< Residual plus measurement/alignment uncertainty, meters.
    std::size_t samples = 0;            ///< Actual qualifying hits used by the fit.
};

struct LocalGroundBuildResult;

/** @brief Owned patch model pinned to one successfully built immutable ESDF.
 * This is a declared cell-resolution sensing model. Four sample quadrants do
 * not prove that subcell holes, transparent surfaces or load-bearing failures
 * are absent. Unknown or unsupported cells cannot be used for execution.
 */
class LocalGroundSnapshot {
public:
    /** @brief Exact immutable instance carrying all original acquisition provenance. */
    const std::shared_ptr<const LocalEsdfSnapshot>& geometry() const { return geometry_; }
    /** @brief Calibrated contract retained unchanged by the builder. */
    const LocalGroundOptions& options() const { return options_; }
    /** @brief Earliest original geometry/calibration expiry; construction cannot renew it. */
    double valid_until() const { return valid_until_; }
    /** @brief Shared-clock publication time. */
    double built_at() const { return built_at_; }
    /** @brief Measured patch or unsupported default for expired/regressed/outside queries. */
    GroundPatch Patch(unsigned x, unsigned y, double now) const;

private:
    /** @brief Only complete validated construction may publish this type. */
    LocalGroundSnapshot() = default;
    friend LocalGroundBuildResult BuildLocalGround(const std::shared_ptr<const LocalEsdfSnapshot>&,
                                                   const LocalGroundOptions&, const ExecutionClock&);
    std::shared_ptr<const LocalEsdfSnapshot> geometry_; ///< Pinned geometry, not just a matching revision.
    LocalGroundOptions options_;              ///< Fixed limits and owner attestation.
    std::vector<GroundPatch> patches_;        ///< X-fast XY model, bounded by the ESDF dimensions.
    double built_at_ = 0.0, valid_until_ = 0.0; ///< Original time bounds, never silently renewed.
};

/** @brief No partial support model is published on invalidity, exceptions or work exhaustion. */
struct LocalGroundBuildResult {
    std::shared_ptr<const LocalGroundSnapshot> snapshot; ///< Complete immutable model on success.
    std::size_t work = 0;                 ///< Samples, cells, fitting and residual checks.
    std::size_t supported_patches = 0;    ///< Actual well-supported cells, not synthesized floor.
    std::string reason;                  ///< Explicit construction rejection cause.
};

/** @brief Fit actual surface hits; missing cells remain unsupported.
 * Sparse/collinear data, conflicting height surfaces, excessive residual or
 * slope do not become floor. No feature token or nominal plane fills a cell.
 */
LocalGroundBuildResult BuildLocalGround(const std::shared_ptr<const LocalEsdfSnapshot>& geometry,
                                       const LocalGroundOptions& options, const ExecutionClock& clock);

/** @brief Check all cells conservatively intersecting the expanded chord capsule.
 * Checks measured reference-height compatibility, adjacent step/drop bounds and
 * expiry. stopping_distance was already included in expanded_body; it is never
 * added twice. All work is reported and bounded by query.maximum_work so the
 * trajectory consumer can charge it to its single aggregate work scope.
 */
GroundSupportVerdict VerifyGroundSupport(const LocalGroundSnapshot& ground,
                                       const GroundSupportQuery& query, const ExecutionClock& clock);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
