#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_INERTIAL_INITIALIZATION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_INERTIAL_INITIALIZATION_HPP_

#include <Eigen/Core>

namespace autonomy::localization::atlas::internal {

using PositionMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

struct MetricGravitySeedDiagnostics {
    double integrated_time_seconds = 0.0;
    double visual_delta_velocity_norm = 0.0;
    double raw_impulse_norm = 0.0;
    double corrected_impulse_norm = 0.0;
    double maximum_fit_residual_meters = 0.0;
    double minimum_reciprocal_condition = 0.0;
};

bool EstimateMetricGravityDirection(
    const Eigen::VectorXd& timestamps, const PositionMatrix& positions,
    const Eigen::Vector3d& raw_gravity_impulse,
    double integrated_time_seconds, Eigen::Vector3d* gravity_direction,
    MetricGravitySeedDiagnostics* diagnostics);

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_INERTIAL_INITIALIZATION_HPP_
