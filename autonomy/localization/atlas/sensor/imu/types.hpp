/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * IMU types adapted from ORB-SLAM3 ImuTypes (double / Google style).
 */

/**
 * @file
 * @brief IMU measurements, calibration, bias, and keyframe-interval `Preintegrator`.
 *
 * Aligned with ORB-SLAM3 `IMU::Preintegrated`: accumulates rotation / velocity / position
 * deltas between keyframes, plus Jacobians w.r.t. gyro/accel bias and covariance for inertial edges.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_TYPES_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_TYPES_HPP_

#include <mutex>
#include <vector>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace imu {

inline constexpr double kGravity = 9.81;  ///< Gravity magnitude (m/s²).

/**
 * @struct autonomy::localization::atlas::sensor::imu::Measurement
 * @brief Single IMU sample (specific force + angular velocity + timestamp).
 */
struct Measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3 acceleration = Vec3::Zero();      ///< Specific force / accel reading (m/s²).
    Vec3 angular_velocity = Vec3::Zero();  ///< Angular velocity (rad/s).
    double timestamp = 0.0;                ///< Timestamp (seconds).

    /**
     * @brief Default all-zero measurement.
     */
    Measurement() = default;

    /**
     * @brief Construct from components.
     * @param ax Accelerometer x.
     * @param ay Accelerometer y.
     * @param az Accelerometer z.
     * @param wx Gyro x.
     * @param wy Gyro y.
     * @param wz Gyro z.
     * @param t Timestamp.
     */
    Measurement(double ax, double ay, double az, double wx, double wy,
                double wz, double t)
        : acceleration(ax, ay, az), angular_velocity(wx, wy, wz), timestamp(t) {}
};

/**
 * @struct autonomy::localization::atlas::sensor::imu::Bias
 * @brief Constant IMU accelerometer and gyro biases.
 */
struct Bias {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3 accelerometer = Vec3::Zero();  ///< Accel bias (m/s²).
    Vec3 gyroscope = Vec3::Zero();      ///< Gyro bias (rad/s).

    /**
     * @brief Default zero bias.
     */
    Bias() = default;

    /**
     * @brief Construct from components.
     * @param bax Accel bias x.
     * @param bay Accel bias y.
     * @param baz Accel bias z.
     * @param bwx Gyro bias x.
     * @param bwy Gyro bias y.
     * @param bwz Gyro bias z.
     */
    Bias(double bax, double bay, double baz, double bwx, double bwy,
         double bwz)
        : accelerometer(bax, bay, baz), gyroscope(bwx, bwy, bwz) {}

    /**
     * @brief Copy values from another bias.
     * @param other Source bias.
     */
    void CopyFrom(const Bias& other) {
        accelerometer = other.accelerometer;
        gyroscope = other.gyroscope;
    }
};

/**
 * @struct autonomy::localization::atlas::sensor::imu::Calib
 * @brief IMU–camera extrinsics and noise / random-walk covariance calibration.
 *
 * `T_body_camera` is body←camera (ORB `Tbc`); `Set` also writes diagonal noise and walk.
 */
struct Calib {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SE3 T_body_camera = SE3Identity();  ///< body ← camera(Tbc).
    SE3 T_camera_body = SE3Identity();  ///< camera ← body(Tcb).
    Eigen::DiagonalMatrix<double, 6> covariance{1, 1, 1, 1, 1, 1};  ///< Measurement noise diagonal (ω×3, a×3).
    Eigen::DiagonalMatrix<double, 6> covariance_walk{1, 1, 1, 1, 1,
                                                     1};  ///< Bias random-walk diagonal.
    bool is_set = false;  ///< Whether `Set` has been called to write calibration.

    /**
     * @brief Write extrinsics and noise densities; mark `is_set`.
     * @param T_bc body←camera extrinsics.
     * @param ng Gyro noise density.
     * @param na Accel noise density.
     * @param ngw Gyro bias random walk.
     * @param naw Accel bias random walk.
     */
    void Set(const SE3& T_bc, double ng, double na, double ngw, double naw) {
        T_body_camera = T_bc;
        T_camera_body = T_bc.inverse();
        const double ng2 = ng * ng;
        const double na2 = na * na;
        const double ngw2 = ngw * ngw;
        const double naw2 = naw * naw;
        covariance.diagonal() << ng2, ng2, ng2, na2, na2, na2;
        covariance_walk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
        is_set = true;
    }
};

/**
 * @struct autonomy::localization::atlas::sensor::imu::IntegratedRotation
 * @brief Single-step gyro integration: rotation delta and right Jacobian (ORB `IntegratedRotation`).
 */
struct IntegratedRotation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double delta_t = 0.0;                   ///< This step time interval.
    Mat33 delta_R = Mat33::Identity();      ///< Rotation increment ΔR.
    Mat33 right_jacobian = Mat33::Identity();  ///< SO(3) right Jacobian.

    /**
     * @brief Default identity rotation.
     */
    IntegratedRotation() = default;

    /**
     * @brief Integrate one step from angular velocity, bias, and dt.
     * @param angular_velocity Gyro reading.
     * @param bias Current gyro bias.
     * @param dt Time interval.
     */
    IntegratedRotation(const Vec3& angular_velocity, const Bias& bias,
                       double dt);
};

/**
 * @class autonomy::localization::atlas::sensor::imu::Preintegrator
 * @brief Keyframe-interval IMU preintegration (ORB-SLAM3 `IMU::Preintegrated`).
 *
 * Under fixed bias, accumulates @f$ \Delta R,\Delta v,\Delta p @f$ plus bias Jacobians and 15-D covariance;
 * after bias updates, `Reintegrate` or first-order `GetDelta*` correction. Thread-safe: public
 * reads/writes are guarded by internal `mutex_`.
 */
class Preintegrator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @struct State
     * @brief Serializable snapshot (ORB `Preintegrated` serialization fields).
     */
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double delta_t = 0.0;  ///< Total integration duration.
        Eigen::Matrix<double, 15, 15> covariance =
            Eigen::Matrix<double, 15, 15>::Zero();  ///< Preintegration covariance.
        Eigen::Matrix<double, 15, 15> information =
            Eigen::Matrix<double, 15, 15>::Zero();  ///< Information matrix (cov inverse).
        Eigen::Matrix<double, 6, 1> nga_diag =
            Eigen::Matrix<double, 6, 1>::Ones();  ///< Measurement noise diagonal.
        Eigen::Matrix<double, 6, 1> nga_walk_diag =
            Eigen::Matrix<double, 6, 1>::Ones();  ///< Random-walk noise diagonal.
        Bias bias;          ///< Original bias used for integration.
        Bias bias_updated;  ///< Updated bias.
        Mat33 dR = Mat33::Identity();  ///< Rotation preintegration.
        Vec3 dV = Vec3::Zero();        ///< Velocity preintegration.
        Vec3 dP = Vec3::Zero();        ///< Position preintegration.
        Mat33 JRg = Mat33::Zero();     ///< ∂ΔR/∂bg.
        Mat33 JVg = Mat33::Zero();     ///< ∂Δv/∂bg.
        Mat33 JVa = Mat33::Zero();     ///< ∂Δv/∂ba.
        Mat33 JPg = Mat33::Zero();     ///< ∂Δp/∂bg.
        Mat33 JPa = Mat33::Zero();     ///< ∂Δp/∂ba.
        Vec3 avg_A = Vec3::Zero();     ///< Mean acceleration (diagnostics).
        Vec3 avg_W = Vec3::Zero();     ///< Mean angular velocity (diagnostics).
        Eigen::Matrix<double, 6, 1> db =
            Eigen::Matrix<double, 6, 1>::Zero();  ///< Bias increment [dbg; dba].
        std::vector<Eigen::Matrix<double, 7, 1>>
            measurements;  ///< Raw integration segments; each row ax,ay,az,wx,wy,wz,dt.
    };

    /**
     * @brief Empty preintegrator (uncalibrated).
     */
    Preintegrator() = default;

    /**
     * @brief Initialize with given bias and calibration.
     * @param bias Initial bias.
     * @param calib Extrinsics and noise calibration.
     */
    Preintegrator(const Bias& bias, const Calib& calib);

    /**
     * @brief Deep-copy constructor.
     * @param other Source preintegrator.
     */
    explicit Preintegrator(const Preintegrator& other);

    /**
     * @brief Reset integrals and restart with a new bias (clears measurement buffer).
     * @param bias New bias.
     */
    void Initialize(const Bias& bias);

    /**
     * @brief Integrate a new measurement segment; update preintegration and covariance.
     * @param acceleration Accel reading.
     * @param angular_velocity Gyro reading.
     * @param dt This segment time interval.
     */
    void IntegrateNewMeasurement(const Vec3& acceleration,
                                 const Vec3& angular_velocity, double dt);

    /**
     * @brief Reintegrate from scratch using buffered measurements and `bias`.
     */
    void Reintegrate();

    /**
     * @brief Merge a previous preintegration segment before this interval start.
     * @param previous Preintegration from the previous keyframe interval.
     */
    void MergePrevious(const Preintegrator& previous);

    /**
     * @brief Set updated bias and record increment `db` relative to original bias.
     * @param bias New bias.
     */
    void SetNewBias(const Bias& bias);

    /**
     * @brief Refresh information matrix from covariance (after MergePrevious / Reintegrate).
     */
    void RefreshInformation();

    /**
     * @brief Export a serializable state snapshot.
     * @return Current `State`.
     */
    State ExportState() const;

    /**
     * @brief Restore internal state from a snapshot.
     * @param state Previously exported state.
     */
    void ImportState(const State& state);

    /**
     * @brief Rotation increment with first-order bias correction.
     * @param bias Query bias.
     * @return ΔR(bias).
     */
    Mat33 GetDeltaRotation(const Bias& bias) const;

    /**
     * @brief Velocity increment with first-order bias correction.
     * @param bias Query bias.
     * @return Δv(bias).
     */
    Vec3 GetDeltaVelocity(const Bias& bias) const;

    /**
     * @brief Position increment with first-order bias correction.
     * @param bias Query bias.
     * @return Δp(bias).
     */
    Vec3 GetDeltaPosition(const Bias& bias) const;

    /**
     * @brief ΔR corrected relative to `bias_updated_`.
     */
    Mat33 GetUpdatedDeltaRotation() const;

    /**
     * @brief Δv corrected relative to `bias_updated_`.
     */
    Vec3 GetUpdatedDeltaVelocity() const;

    /**
     * @brief Δp corrected relative to `bias_updated_`.
     */
    Vec3 GetUpdatedDeltaPosition() const;

    /**
     * @brief ΔR under original bias (no first-order correction).
     */
    Mat33 GetOriginalDeltaRotation() const;

    /**
     * @brief Δv under original bias.
     */
    Vec3 GetOriginalDeltaVelocity() const;

    /**
     * @brief Δp under original bias.
     */
    Vec3 GetOriginalDeltaPosition() const;

    /**
     * @brief Original bias used for integration.
     */
    Bias GetOriginalBias() const;

    /**
     * @brief Bias most recently written by `SetNewBias`.
     */
    Bias GetUpdatedBias() const;

    /**
     * @brief Updated-bias increment relative to original `[dbg; dba]`.
     */
    Eigen::Matrix<double, 6, 1> GetDeltaBias() const;

    double delta_t = 0.0;  ///< Total integration duration.
    Eigen::Matrix<double, 15, 15> covariance =
        Eigen::Matrix<double, 15, 15>::Zero();  ///< 15-D preintegration covariance.
    Eigen::Matrix<double, 15, 15> information =
        Eigen::Matrix<double, 15, 15>::Zero();  ///< Information matrix.
    Eigen::DiagonalMatrix<double, 6> Nga{1, 1, 1, 1, 1, 1};  ///< Measurement noise.
    Eigen::DiagonalMatrix<double, 6> NgaWalk{1, 1, 1, 1, 1, 1};  ///< Random-walk noise.

    Bias bias;                      ///< Original integration bias.
    Mat33 dR = Mat33::Identity();   ///< Rotation preintegration.
    Vec3 dV = Vec3::Zero();         ///< Velocity preintegration.
    Vec3 dP = Vec3::Zero();         ///< Position preintegration.
    Mat33 JRg = Mat33::Zero();      ///< ∂ΔR/∂bg.
    Mat33 JVg = Mat33::Zero();      ///< ∂Δv/∂bg.
    Mat33 JVa = Mat33::Zero();      ///< ∂Δv/∂ba.
    Mat33 JPg = Mat33::Zero();      ///< ∂Δp/∂bg.
    Mat33 JPa = Mat33::Zero();      ///< ∂Δp/∂ba.
    Vec3 avg_A = Vec3::Zero();      ///< Mean acceleration.
    Vec3 avg_W = Vec3::Zero();      ///< Mean angular velocity.

private:
    /**
     * @struct Integrable
     * @brief Single integrable measurement segment in the buffer.
     */
    struct Integrable {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vec3 a = Vec3::Zero();  ///< Acceleration.
        Vec3 w = Vec3::Zero();  ///< Angular velocity.
        double t = 0.0;         ///< Time interval dt.
    };

    /**
     * @brief Orthogonalize / normalize a rotation matrix.
     * @param R Input rotation.
     * @return Normalized rotation.
     */
    static Mat33 NormalizeRotation(const Mat33& R);

    Bias bias_updated_;  ///< Updated bias.
    Eigen::Matrix<double, 6, 1> db_ =
        Eigen::Matrix<double, 6, 1>::Zero();  ///< Bias increment.
    std::vector<Integrable> measurements_;   ///< Measurement buffer (for reintegration).
    mutable std::mutex mutex_;               ///< Guards concurrent read/write.
};

}  // namespace imu
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_TYPES_HPP_
