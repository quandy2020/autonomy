/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_TYPES_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>
#include "autolink/common/log.hpp"
#include "autonomy/common/math/se3.hpp"

#include "atlas/common/io/serialization_utils.hpp"

namespace autonomy::localization::atlas {

namespace IMU {

const float GRAVITY_VALUE = 9.81;

// IMU measurement (gyro, accelerometer and timestamp)
class Point
{
public:
    Point(const float& acc_x, const float& acc_y, const float& acc_z,
          const float& ang_vel_x, const float& ang_vel_y,
          const float& ang_vel_z, const double& timestamp)
        : a(acc_x, acc_y, acc_z),
          w(ang_vel_x, ang_vel_y, ang_vel_z),
          t(timestamp) {}
    Point(const cv::Point3f Acc, const cv::Point3f Gyro,
          const double& timestamp)
        : a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), t(timestamp) {}

public:
    Eigen::Vector3f a;
    Eigen::Vector3f w;
    double t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// IMU biases (gyro and accelerometer)
class Bias
{
    friend class ::autonomy::localization::atlas::ArchiveAccess;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& bax;
        ar& bay;
        ar& baz;

        ar& bwx;
        ar& bwy;
        ar& bwz;
    }

public:
    Bias() : bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0) {}
    Bias(const float& b_acc_x, const float& b_acc_y, const float& b_acc_z,
         const float& b_ang_vel_x, const float& b_ang_vel_y,
         const float& b_ang_vel_z)
        : bax(b_acc_x),
          bay(b_acc_y),
          baz(b_acc_z),
          bwx(b_ang_vel_x),
          bwy(b_ang_vel_y),
          bwz(b_ang_vel_z) {}
    void CopyFrom(Bias& b);
    friend std::ostream& operator<<(std::ostream& out, const Bias& b);

public:
    float bax, bay, baz;
    float bwx, bwy, bwz;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// IMU calibration (Tbc, Tcb, noise)
class Calib
{
    friend class ::autonomy::localization::atlas::ArchiveAccess;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        serializeSE3(ar, mTcb, version);
        serializeSE3(ar, mTbc, version);

        ar& MakeArray(Cov.diagonal().data(), Cov.diagonal().size());
        ar& MakeArray(CovWalk.diagonal().data(), CovWalk.diagonal().size());

        ar& mbIsSet;
    }

public:
    Calib(const autonomy::common::math::SE3<float>& Tbc, const float& ng,
          const float& na, const float& ngw, const float& naw) {
        Set(Tbc, ng, na, ngw, naw);
    }

    Calib(const Calib& calib);
    Calib() {
        mbIsSet = false;
    }

    // void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const
    // float &ngw, const float &naw);
    void Set(const autonomy::common::math::SE3<float>& sophTbc, const float& ng,
             const float& na, const float& ngw, const float& naw);

public:
    // common Lie-group math implementation
    autonomy::common::math::SE3<float> mTcb;
    autonomy::common::math::SE3<float> mTbc;
    Eigen::DiagonalMatrix<float, 6> Cov, CovWalk;
    bool mbIsSet;
};

// Integration of 1 gyro measurement
class IntegratedRotation
{
public:
    IntegratedRotation() {}
    IntegratedRotation(const Eigen::Vector3f& angVel, const Bias& imuBias,
                       const float& time);

public:
    float deltaT;  // integration time
    Eigen::Matrix3f deltaR;
    Eigen::Matrix3f rightJ;  // right jacobian
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Preintegration of Imu Measurements
class Preintegrated
{
    friend class ::autonomy::localization::atlas::ArchiveAccess;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& dT;
        ar& MakeArray(C.data(), C.size());
        ar& MakeArray(Info.data(), Info.size());
        ar& MakeArray(Nga.diagonal().data(), Nga.diagonal().size());
        ar& MakeArray(NgaWalk.diagonal().data(), NgaWalk.diagonal().size());
        ar& b;
        ar& MakeArray(dR.data(), dR.size());
        ar& MakeArray(dV.data(), dV.size());
        ar& MakeArray(dP.data(), dP.size());
        ar& MakeArray(JRg.data(), JRg.size());
        ar& MakeArray(JVg.data(), JVg.size());
        ar& MakeArray(JVa.data(), JVa.size());
        ar& MakeArray(JPg.data(), JPg.size());
        ar& MakeArray(JPa.data(), JPa.size());
        ar& MakeArray(avgA.data(), avgA.size());
        ar& MakeArray(avgW.data(), avgW.size());

        ar& bu;
        ar& MakeArray(db.data(), db.size());
        ar& mvMeasurements;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Preintegrated(const Bias& b_, const Calib& calib);
    Preintegrated(Preintegrated* pImuPre);
    Preintegrated() {}
    ~Preintegrated() {}
    void CopyFrom(Preintegrated* pImuPre);
    void Initialize(const Bias& b_);
    void IntegrateNewMeasurement(const Eigen::Vector3f& acceleration,
                                 const Eigen::Vector3f& angVel,
                                 const float& dt);
    void Reintegrate();
    void MergePrevious(Preintegrated* pPrev);
    void SetNewBias(const Bias& bu_);
    IMU::Bias GetDeltaBias(const Bias& b_);

    Eigen::Matrix3f GetDeltaRotation(const Bias& b_);
    Eigen::Vector3f GetDeltaVelocity(const Bias& b_);
    Eigen::Vector3f GetDeltaPosition(const Bias& b_);

    Eigen::Matrix3f GetUpdatedDeltaRotation();
    Eigen::Vector3f GetUpdatedDeltaVelocity();
    Eigen::Vector3f GetUpdatedDeltaPosition();

    Eigen::Matrix3f GetOriginalDeltaRotation();
    Eigen::Vector3f GetOriginalDeltaVelocity();
    Eigen::Vector3f GetOriginalDeltaPosition();

    Eigen::Matrix<float, 6, 1> GetDeltaBias();

    Bias GetOriginalBias();
    Bias GetUpdatedBias();

    void printMeasurements() const {
        AINFO << "pint meas:\n";
        for (std::size_t i = 0; i < mvMeasurements.size(); ++i) {
            AINFO << "meas " << mvMeasurements[i].t << std::endl;
        }
        AINFO << "end pint meas:\n";
    }

public:
    float dT;
    Eigen::Matrix<float, 15, 15> C;
    Eigen::Matrix<float, 15, 15> Info;
    Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    Bias b;
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    Eigen::Vector3f avgA, avgW;

private:
    // Updated bias
    Bias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    Eigen::Matrix<float, 6, 1> db;

    struct integrable {
        template <class Archive>
        void serialize(Archive& ar, const unsigned int version) {
            ar& MakeArray(a.data(), a.size());
            ar& MakeArray(w.data(), w.size());
            ar& t;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        integrable() {}
        integrable(const Eigen::Vector3f& a_, const Eigen::Vector3f& w_,
                   const float& t_)
            : a(a_), w(w_), t(t_) {}
        Eigen::Vector3f a, w;
        float t;
    };

    std::vector<integrable> mvMeasurements;

    std::mutex mMutex;
};

// Lie Algebra Functions
Eigen::Matrix3f RightJacobianSO3(const float& x, const float& y,
                                 const float& z);
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f& v);

Eigen::Matrix3f InverseRightJacobianSO3(const float& x, const float& y,
                                        const float& z);
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f& v);

Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f& R);

}  // namespace IMU

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_TYPES_HPP_
