#include "atlas/mapping/inertial_initialization.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

#include "atlas/sensor/imu_types.hpp"

namespace autonomy::localization::atlas::internal {
namespace {

constexpr int kMaximumEndpointFitSamples = 5;
constexpr int kMinimumEndpointFitSamples = 4;
constexpr double kMinimumEndpointFitSpanSeconds = 0.2;
constexpr double kMinimumReciprocalCondition = 1.0e-3;
constexpr double kMaximumFitResidualMeters = 0.15;
constexpr double kMaximumVisualAccelerationInGravities = 2.0;
constexpr double kMinimumGravityImpulseRatio = 0.5;
constexpr double kMaximumGravityImpulseRatio = 1.5;

struct VelocityFit {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double residual_meters = 0.0;
    double reciprocal_condition = 0.0;
};

bool FitEndpointVelocity(const Eigen::VectorXd& timestamps,
                         const PositionMatrix& positions, int begin, int count,
                         bool evaluate_at_end, VelocityFit* fit) {
    if (fit == nullptr || begin < 0 || count < kMinimumEndpointFitSamples ||
        begin + count > timestamps.size() ||
        positions.rows() != timestamps.size()) {
        return false;
    }

    const double first_time = timestamps[begin];
    const double span = timestamps[begin + count - 1] - first_time;
    if (!std::isfinite(first_time) || !std::isfinite(span) ||
        span < kMinimumEndpointFitSpanSeconds) {
        return false;
    }

    Eigen::MatrixXd design(count, 3);
    PositionMatrix samples(count, 3);
    for (int index = 0; index < count; ++index) {
        const double normalized_time =
            (timestamps[begin + index] - first_time) / span;
        if (!std::isfinite(normalized_time) || normalized_time < 0.0 ||
            normalized_time > 1.0) {
            return false;
        }
        design(index, 0) = 1.0;
        design(index, 1) = normalized_time;
        design(index, 2) = normalized_time * normalized_time;
        samples.row(index) = positions.row(begin + index);
    }
    if (!samples.allFinite())
        return false;

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        design, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues().size() != 3)
        return false;
    const double largest_singular_value = svd.singularValues()[0];
    const double smallest_singular_value = svd.singularValues()[2];
    if (!std::isfinite(largest_singular_value) ||
        !std::isfinite(smallest_singular_value) ||
        largest_singular_value <= 0.0) {
        return false;
    }

    fit->reciprocal_condition =
        smallest_singular_value / largest_singular_value;
    if (fit->reciprocal_condition < kMinimumReciprocalCondition)
        return false;

    const Eigen::Matrix<double, 3, 3> coefficients = svd.solve(samples);
    const double normalized_endpoint = evaluate_at_end ? 1.0 : 0.0;
    fit->velocity =
        (coefficients.row(1) +
         2.0 * normalized_endpoint * coefficients.row(2))
            .transpose() /
        span;
    fit->residual_meters =
        std::sqrt((design * coefficients - samples).squaredNorm() /
                  static_cast<double>(count));
    return fit->velocity.allFinite() &&
           std::isfinite(fit->residual_meters) &&
           fit->residual_meters <= kMaximumFitResidualMeters;
}

}  // namespace

bool EstimateMetricGravityDirection(
    const Eigen::VectorXd& timestamps, const PositionMatrix& positions,
    const Eigen::Vector3d& raw_gravity_impulse,
    double integrated_time_seconds, Eigen::Vector3d* gravity_direction,
    MetricGravitySeedDiagnostics* diagnostics) {
    if (gravity_direction == nullptr || diagnostics == nullptr)
        return false;
    *diagnostics = MetricGravitySeedDiagnostics{};
    diagnostics->integrated_time_seconds = integrated_time_seconds;
    diagnostics->raw_impulse_norm = raw_gravity_impulse.norm();

    const int sample_count = timestamps.size();
    if (sample_count < 2 * kMinimumEndpointFitSamples ||
        positions.rows() != sample_count || positions.cols() != 3 ||
        !timestamps.allFinite() || !positions.allFinite() ||
        !raw_gravity_impulse.allFinite() ||
        !std::isfinite(integrated_time_seconds) ||
        integrated_time_seconds <= 0.0) {
        return false;
    }

    const double keyframe_time_span =
        timestamps[sample_count - 1] - timestamps[0];
    const double maximum_time_error =
        std::max(0.02, 0.05 * keyframe_time_span);
    if (!std::isfinite(keyframe_time_span) || keyframe_time_span <= 0.0 ||
        std::abs(integrated_time_seconds - keyframe_time_span) >
            maximum_time_error) {
        return false;
    }

    const int fit_sample_count =
        std::min(kMaximumEndpointFitSamples, sample_count / 2);
    VelocityFit start_fit;
    VelocityFit end_fit;
    if (!FitEndpointVelocity(timestamps, positions, 0, fit_sample_count, false,
                             &start_fit) ||
        !FitEndpointVelocity(timestamps, positions,
                             sample_count - fit_sample_count, fit_sample_count,
                             true, &end_fit)) {
        return false;
    }

    const Eigen::Vector3d visual_delta_velocity =
        end_fit.velocity - start_fit.velocity;
    diagnostics->visual_delta_velocity_norm = visual_delta_velocity.norm();
    diagnostics->maximum_fit_residual_meters =
        std::max(start_fit.residual_meters, end_fit.residual_meters);
    diagnostics->minimum_reciprocal_condition =
        std::min(start_fit.reciprocal_condition,
                 end_fit.reciprocal_condition);

    const double expected_gravity_impulse =
        IMU::GRAVITY_VALUE * integrated_time_seconds;
    if (!visual_delta_velocity.allFinite() ||
        visual_delta_velocity.norm() >
            kMaximumVisualAccelerationInGravities *
                expected_gravity_impulse) {
        return false;
    }

    const Eigen::Vector3d corrected_gravity_impulse =
        raw_gravity_impulse + visual_delta_velocity;
    diagnostics->corrected_impulse_norm =
        corrected_gravity_impulse.norm();
    if (!corrected_gravity_impulse.allFinite() ||
        corrected_gravity_impulse.norm() <
            kMinimumGravityImpulseRatio * expected_gravity_impulse ||
        corrected_gravity_impulse.norm() >
            kMaximumGravityImpulseRatio * expected_gravity_impulse) {
        return false;
    }

    *gravity_direction = corrected_gravity_impulse.normalized();
    return gravity_direction->allFinite();
}

}  // namespace autonomy::localization::atlas::internal
