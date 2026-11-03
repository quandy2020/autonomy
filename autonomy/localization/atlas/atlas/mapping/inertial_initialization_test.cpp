#include "atlas/mapping/inertial_initialization.hpp"

#include <gtest/gtest.h>

#include "atlas/sensor/imu_types.hpp"

namespace autonomy::localization::atlas::internal {
namespace {

TEST(InertialInitializationTest,
     MetricGravitySeedCompensatesNonuniformConstantAcceleration) {
    const Eigen::VectorXd timestamps =
        (Eigen::VectorXd(10) << 0.0, 0.11, 0.25, 0.42, 0.61, 0.83, 1.08,
         1.36, 1.67, 2.01)
            .finished();
    const Eigen::Vector3d initial_position(1.0, -2.0, 0.5);
    const Eigen::Vector3d initial_velocity(2.0, -1.0, 0.3);
    const Eigen::Vector3d acceleration(1.5, -0.8, 0.4);
    PositionMatrix positions(timestamps.size(), 3);
    for (int index = 0; index < timestamps.size(); ++index) {
        const double time = timestamps[index];
        positions.row(index) =
            (initial_position + initial_velocity * time +
             0.5 * acceleration * time * time)
                .transpose();
    }

    const double integrated_time =
        timestamps[timestamps.size() - 1] - timestamps[0];
    const Eigen::Vector3d gravity_impulse(
        0.0, 0.0, -IMU::GRAVITY_VALUE * integrated_time);
    const Eigen::Vector3d visual_delta_velocity =
        acceleration * integrated_time;
    const Eigen::Vector3d raw_gravity_impulse =
        gravity_impulse - visual_delta_velocity;

    Eigen::Vector3d gravity_direction;
    MetricGravitySeedDiagnostics diagnostics;
    ASSERT_TRUE(EstimateMetricGravityDirection(
        timestamps, positions, raw_gravity_impulse, integrated_time,
        &gravity_direction, &diagnostics));
    EXPECT_NEAR(gravity_direction.x(), 0.0, 1.0e-9);
    EXPECT_NEAR(gravity_direction.y(), 0.0, 1.0e-9);
    EXPECT_NEAR(gravity_direction.z(), -1.0, 1.0e-9);
    EXPECT_NEAR(diagnostics.visual_delta_velocity_norm,
                visual_delta_velocity.norm(), 1.0e-9);
}

TEST(InertialInitializationTest, MetricGravitySeedHandlesConstantVelocity) {
    const Eigen::VectorXd timestamps =
        Eigen::VectorXd::LinSpaced(10, 0.0, 1.8);
    PositionMatrix positions(timestamps.size(), 3);
    const Eigen::Vector3d velocity(0.4, -0.2, 0.1);
    for (int index = 0; index < timestamps.size(); ++index)
        positions.row(index) = (velocity * timestamps[index]).transpose();

    const double integrated_time = 1.8;
    const Eigen::Vector3d raw_gravity_impulse(
        0.0, 0.0, -IMU::GRAVITY_VALUE * integrated_time);
    Eigen::Vector3d gravity_direction;
    MetricGravitySeedDiagnostics diagnostics;
    ASSERT_TRUE(EstimateMetricGravityDirection(
        timestamps, positions, raw_gravity_impulse, integrated_time,
        &gravity_direction, &diagnostics));
    EXPECT_NEAR(gravity_direction.z(), -1.0, 1.0e-9);
    EXPECT_NEAR(diagnostics.visual_delta_velocity_norm, 0.0, 1.0e-9);
}

TEST(InertialInitializationTest, MetricGravitySeedRejectsInvalidTiming) {
    Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(10);
    PositionMatrix positions = PositionMatrix::Zero(10, 3);
    Eigen::Vector3d gravity_direction;
    MetricGravitySeedDiagnostics diagnostics;
    EXPECT_FALSE(EstimateMetricGravityDirection(
        timestamps, positions, Eigen::Vector3d(0.0, 0.0, -9.81), 1.0,
        &gravity_direction, &diagnostics));
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
