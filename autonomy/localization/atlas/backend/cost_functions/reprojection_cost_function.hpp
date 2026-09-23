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
 */

/**
 * @file reprojection_cost_function.hpp
 * @brief Visual reprojection cost functors (pose-only and BA; mono/stereo).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_REPROJECTION_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_REPROJECTION_COST_FUNCTION_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "Eigen/Core"

#include "autonomy/localization/atlas/backend/cost_functions/cost_helpers.hpp"
#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {


/**
 * @struct autonomy::localization::atlas::backend::MonoPoseOnlyCostFunctor
 * @brief Monocular pose-only reprojection (ORB EdgeSE3ProjectXYZOnlyPose).
 *
 * Residual dimension: 2. Parameter block: pose(6) = \(T_{cw}\) as
 * `[angle_axis, t]`. The world point is fixed.
 */
struct MonoPoseOnlyCostFunctor {
    /**
     * @brief Construct with full ReprojCam intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param cam Camera intrinsics.
     * @param sqrt_info Square-root information weight.
     */
    MonoPoseOnlyCostFunctor(double observed_u, double observed_v, double xw,
                            double yw, double zw, const ReprojCam& cam,
                            double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          cam_(cam),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     */
    MonoPoseOnlyCostFunctor(double observed_u, double observed_v, double xw,
                            double yw, double zw, double fx, double fy,
                            double cx, double cy, double sqrt_info)
        : MonoPoseOnlyCostFunctor(observed_u, observed_v, xw, yw, zw,
                                  ReprojCam::Pinhole(fx, fy, cx, cy),
                                  sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose \(T_{cw}\) parameter block `[aa(3), t(3)]`.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        T point_world[3] = {T(xw_), T(yw_), T(zw_)};
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point_world, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction (owned by Ceres Problem).
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info) {
        return Create(observed_u, observed_v, xw, yw, zw,
                      ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info);
    }

    /**
     * @brief Factory with ReprojCam.
     * @return Heap-allocated AutoDiffCostFunction residual 2, block 6.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       const ReprojCam& cam, double sqrt_info) {
        return new ceres::AutoDiffCostFunction<MonoPoseOnlyCostFunctor, 2, 6>(
            new MonoPoseOnlyCostFunctor(observed_u, observed_v, xw, yw, zw, cam,
                                        sqrt_info));
    }

    double observed_u_;
    double observed_v_;
    double xw_;
    double yw_;
    double zw_;
    ReprojCam cam_;
    double sqrt_info_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoPoseOnlyFromTwbCostFunctor
 * @brief Monocular pose-only cost with pose parameter \(T_{wb}\):
 *        \(T_{cw}=T_{cb} T_{wb}^{-1}\).
 *
 * Residual dimension: 2. Parameter block: pose_wb(6). Extrinsics
 * \(R_{cb},t_{cb}\) are fixed.
 */
struct MonoPoseOnlyFromTwbCostFunctor {
    /**
     * @brief Construct with ReprojCam and body-to-camera extrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param cam Camera intrinsics.
     * @param sqrt_info Square-root information weight.
     * @param R_cb Rotation \(R_{cb}\).
     * @param t_cb Translation \(t_{cb}\).
     */
    MonoPoseOnlyFromTwbCostFunctor(double observed_u, double observed_v,
                                   double xw, double yw, double zw,
                                   const ReprojCam& cam, double sqrt_info,
                                   const Mat33& R_cb, const Vec3& t_cb)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          cam_(cam),
          sqrt_info_(sqrt_info),
          R_cb_(R_cb),
          t_cb_(t_cb) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     * @param R_cb Rotation \(R_{cb}\).
     * @param t_cb Translation \(t_{cb}\).
     */
    MonoPoseOnlyFromTwbCostFunctor(double observed_u, double observed_v,
                                   double xw, double yw, double zw, double fx,
                                   double fy, double cx, double cy,
                                   double sqrt_info, const Mat33& R_cb,
                                   const Vec3& t_cb)
        : MonoPoseOnlyFromTwbCostFunctor(
              observed_u, observed_v, xw, yw, zw,
              ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_cb, t_cb) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_wb \(T_{wb}\) parameter block.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_wb, T* residuals) const {
        T Rwb[9];
        ceres::AngleAxisToRotationMatrix(pose_wb, Rwb);
        const T dx = -Rwb[0] * pose_wb[3] - Rwb[1] * pose_wb[4] -
                     Rwb[2] * pose_wb[5];
        const T dy = -Rwb[3] * pose_wb[3] - Rwb[4] * pose_wb[4] -
                     Rwb[5] * pose_wb[5];
        const T dz = -Rwb[6] * pose_wb[3] - Rwb[7] * pose_wb[4] -
                     Rwb[8] * pose_wb[5];
        const T Rbw00 = Rwb[0], Rbw01 = Rwb[3], Rbw02 = Rwb[6];
        const T Rbw10 = Rwb[1], Rbw11 = Rwb[4], Rbw12 = Rwb[7];
        const T Rbw20 = Rwb[2], Rbw21 = Rwb[5], Rbw22 = Rwb[8];
        const Eigen::Matrix<T, 3, 3> Rcb = R_cb_.cast<T>();
        const Eigen::Matrix<T, 3, 1> tcb = t_cb_.cast<T>();
        Eigen::Matrix<T, 3, 3> Rbw;
        Rbw << Rbw00, Rbw01, Rbw02, Rbw10, Rbw11, Rbw12, Rbw20, Rbw21, Rbw22;
        const Eigen::Matrix<T, 3, 3> Rcw = Rcb * Rbw;
        const Eigen::Matrix<T, 3, 1> tbw(dx, dy, dz);
        const Eigen::Matrix<T, 3, 1> tcw = Rcb * tbw + tcb;
        const Eigen::Matrix<T, 3, 1> pw(T(xw_), T(yw_), T(zw_));
        const Eigen::Matrix<T, 3, 1> pc = Rcw * pw + tcw;
        T point_camera[3] = {pc[0], pc[1], pc[2]};
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info,
                                       const Mat33& R_cb, const Vec3& t_cb) {
        return Create(observed_u, observed_v, xw, yw, zw,
                      ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_cb,
                      t_cb);
    }

    /**
     * @brief Factory with ReprojCam.
     * @return Heap-allocated AutoDiffCostFunction residual 2, block 6.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       const ReprojCam& cam, double sqrt_info,
                                       const Mat33& R_cb, const Vec3& t_cb) {
        return new ceres::AutoDiffCostFunction<MonoPoseOnlyFromTwbCostFunctor, 2,
                                               6>(
            new MonoPoseOnlyFromTwbCostFunctor(observed_u, observed_v, xw, yw,
                                               zw, cam, sqrt_info, R_cb, t_cb));
    }

    double observed_u_;
    double observed_v_;
    double xw_;
    double yw_;
    double zw_;
    ReprojCam cam_;
    double sqrt_info_;
    Mat33 R_cb_;
    Vec3 t_cb_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoPoseOnlyToBodyCostFunctor
 * @brief Right-camera pose-only cost (ORB EdgeSE3ProjectXYZOnlyPoseToBody).
 *
 * Left-camera point is transformed by fixed \(T_{rl}\):
 * \(X_{c2}=R_{rl} X_{c1}+t_{rl}\) then projected.
 * Residual dimension: 2. Parameter block: pose(6) = left-camera \(T_{cw}\).
 */
struct MonoPoseOnlyToBodyCostFunctor {
    /**
     * @brief Construct with ReprojCam and left-to-right extrinsics.
     * @param observed_u Observed pixel u on the right camera.
     * @param observed_v Observed pixel v on the right camera.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param cam Right-camera intrinsics.
     * @param sqrt_info Square-root information weight.
     * @param R_rl Rotation \(R_{rl}\).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoPoseOnlyToBodyCostFunctor(double observed_u, double observed_v,
                                  double xw, double yw, double zw,
                                  const ReprojCam& cam, double sqrt_info,
                                  const Mat33& R_rl, const Vec3& t_rl)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          cam_(cam),
          sqrt_info_(sqrt_info),
          R_rl_(R_rl),
          t_rl_(t_rl) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     * @param R_rl Rotation \(R_{rl}\).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoPoseOnlyToBodyCostFunctor(double observed_u, double observed_v,
                                  double xw, double yw, double zw, double fx,
                                  double fy, double cx, double cy,
                                  double sqrt_info, const Mat33& R_rl,
                                  const Vec3& t_rl)
        : MonoPoseOnlyToBodyCostFunctor(
              observed_u, observed_v, xw, yw, zw,
              ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_rl, t_rl) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose Left-camera \(T_{cw}\) parameter block.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        T point_world[3] = {T(xw_), T(yw_), T(zw_)};
        T point_left[3];
        ceres::AngleAxisRotatePoint(pose, point_world, point_left);
        point_left[0] += pose[3];
        point_left[1] += pose[4];
        point_left[2] += pose[5];

        T point_camera[3];
        point_camera[0] = T(R_rl_(0, 0)) * point_left[0] +
                          T(R_rl_(0, 1)) * point_left[1] +
                          T(R_rl_(0, 2)) * point_left[2] + T(t_rl_.x());
        point_camera[1] = T(R_rl_(1, 0)) * point_left[0] +
                          T(R_rl_(1, 1)) * point_left[1] +
                          T(R_rl_(1, 2)) * point_left[2] + T(t_rl_.y());
        point_camera[2] = T(R_rl_(2, 0)) * point_left[0] +
                          T(R_rl_(2, 1)) * point_left[1] +
                          T(R_rl_(2, 2)) * point_left[2] + T(t_rl_.z());
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return Create(observed_u, observed_v, xw, yw, zw,
                      ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_rl,
                      t_rl);
    }

    /**
     * @brief Factory with ReprojCam: residual 2, block 6.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       const ReprojCam& cam, double sqrt_info,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return new ceres::AutoDiffCostFunction<MonoPoseOnlyToBodyCostFunctor, 2,
                                               6>(
            new MonoPoseOnlyToBodyCostFunctor(observed_u, observed_v, xw, yw,
                                              zw, cam, sqrt_info, R_rl, t_rl));
    }

    double observed_u_;
    double observed_v_;
    double xw_;
    double yw_;
    double zw_;
    ReprojCam cam_;
    double sqrt_info_;
    Mat33 R_rl_;
    Vec3 t_rl_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoPoseOnlyToBodyFromTwbCostFunctor
 * @brief Right-camera pose-only cost (\(T_{wb}\)):
 *        \(X_{c2}=T_{rl} T_{cw}(T_{wb}) X_w\).
 *
 * Residual dimension: 2. Parameter block: pose_wb(6). \(T_{cb}\) and \(T_{rl}\)
 * are fixed.
 */
struct MonoPoseOnlyToBodyFromTwbCostFunctor {
    /**
     * @brief Construct with ReprojCam and extrinsics.
     * @param observed_u Observed pixel u on the right camera.
     * @param observed_v Observed pixel v on the right camera.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param cam Right-camera intrinsics.
     * @param sqrt_info Square-root information weight.
     * @param R_cb Rotation \(R_{cb}\).
     * @param t_cb Translation \(t_{cb}\).
     * @param R_rl Rotation \(R_{rl}\) (left to right).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoPoseOnlyToBodyFromTwbCostFunctor(double observed_u, double observed_v,
                                         double xw, double yw, double zw,
                                         const ReprojCam& cam, double sqrt_info,
                                         const Mat33& R_cb, const Vec3& t_cb,
                                         const Mat33& R_rl, const Vec3& t_rl)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          cam_(cam),
          sqrt_info_(sqrt_info),
          R_cb_(R_cb),
          t_cb_(t_cb),
          R_rl_(R_rl),
          t_rl_(t_rl) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     * @param R_cb Rotation \(R_{cb}\).
     * @param t_cb Translation \(t_{cb}\).
     * @param R_rl Rotation \(R_{rl}\).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoPoseOnlyToBodyFromTwbCostFunctor(double observed_u, double observed_v,
                                         double xw, double yw, double zw,
                                         double fx, double fy, double cx,
                                         double cy, double sqrt_info,
                                         const Mat33& R_cb, const Vec3& t_cb,
                                         const Mat33& R_rl, const Vec3& t_rl)
        : MonoPoseOnlyToBodyFromTwbCostFunctor(
              observed_u, observed_v, xw, yw, zw,
              ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_cb, t_cb, R_rl,
              t_rl) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_wb \(T_{wb}\) parameter block.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_wb, T* residuals) const {
        T Rwb[9];
        ceres::AngleAxisToRotationMatrix(pose_wb, Rwb);
        const T dx = -Rwb[0] * pose_wb[3] - Rwb[1] * pose_wb[4] -
                     Rwb[2] * pose_wb[5];
        const T dy = -Rwb[3] * pose_wb[3] - Rwb[4] * pose_wb[4] -
                     Rwb[5] * pose_wb[5];
        const T dz = -Rwb[6] * pose_wb[3] - Rwb[7] * pose_wb[4] -
                     Rwb[8] * pose_wb[5];
        const T Rbw00 = Rwb[0], Rbw01 = Rwb[3], Rbw02 = Rwb[6];
        const T Rbw10 = Rwb[1], Rbw11 = Rwb[4], Rbw12 = Rwb[7];
        const T Rbw20 = Rwb[2], Rbw21 = Rwb[5], Rbw22 = Rwb[8];
        const Eigen::Matrix<T, 3, 3> Rcb = R_cb_.cast<T>();
        const Eigen::Matrix<T, 3, 1> tcb = t_cb_.cast<T>();
        Eigen::Matrix<T, 3, 3> Rbw;
        Rbw << Rbw00, Rbw01, Rbw02, Rbw10, Rbw11, Rbw12, Rbw20, Rbw21, Rbw22;
        const Eigen::Matrix<T, 3, 3> Rcw = Rcb * Rbw;
        const Eigen::Matrix<T, 3, 1> tbw(dx, dy, dz);
        const Eigen::Matrix<T, 3, 1> tcw = Rcb * tbw + tcb;
        const Eigen::Matrix<T, 3, 1> pw(T(xw_), T(yw_), T(zw_));
        const Eigen::Matrix<T, 3, 1> pc_left = Rcw * pw + tcw;
        const Eigen::Matrix<T, 3, 3> Rrl = R_rl_.cast<T>();
        const Eigen::Matrix<T, 3, 1> trl = t_rl_.cast<T>();
        const Eigen::Matrix<T, 3, 1> pc = Rrl * pc_left + trl;
        T point_camera[3] = {pc[0], pc[1], pc[2]};
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info,
                                       const Mat33& R_cb, const Vec3& t_cb,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return Create(observed_u, observed_v, xw, yw, zw,
                      ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_cb, t_cb,
                      R_rl, t_rl);
    }

    /**
     * @brief Factory with ReprojCam.
     * @return Heap-allocated AutoDiffCostFunction residual 2, block 6.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xw, double yw, double zw,
                                       const ReprojCam& cam, double sqrt_info,
                                       const Mat33& R_cb, const Vec3& t_cb,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return new ceres::AutoDiffCostFunction<
            MonoPoseOnlyToBodyFromTwbCostFunctor, 2, 6>(
            new MonoPoseOnlyToBodyFromTwbCostFunctor(
                observed_u, observed_v, xw, yw, zw, cam, sqrt_info, R_cb, t_cb,
                R_rl, t_rl));
    }

    double observed_u_;
    double observed_v_;
    double xw_;
    double yw_;
    double zw_;
    ReprojCam cam_;
    double sqrt_info_;
    Mat33 R_cb_;
    Vec3 t_cb_;
    Mat33 R_rl_;
    Vec3 t_rl_;
};

/**
 * @struct autonomy::localization::atlas::backend::StereoPoseOnlyCostFunctor
 * @brief Stereo pose-only reprojection (g2o EdgeStereoSE3ProjectXYZOnlyPose).
 *
 * Residual dimension: 3 = \((u,v,u_r)\). Parameter block: pose(6) = \(T_{cw}\).
 */
struct StereoPoseOnlyCostFunctor {
    /**
     * @brief Construct stereo pose-only edge.
     * @param observed_u Left observed u.
     * @param observed_v Left observed v.
     * @param observed_ur Right observed u.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param bf Baseline times focal length.
     * @param sqrt_info Square-root information weight.
     */
    StereoPoseOnlyCostFunctor(double observed_u, double observed_v,
                              double observed_ur, double xw, double yw,
                              double zw, double fx, double fy, double cx,
                              double cy, double bf, double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          observed_ur_(observed_ur),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          bf_(bf),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose \(T_{cw}\) parameter block.
     * @param[out] residuals Length-3 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        T point_world[3] = {T(xw_), T(yw_), T(zw_)};
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point_world, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];
        if (point_camera[2] <= T(1e-6)) {
            residuals[0] = residuals[1] = residuals[2] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / point_camera[2];
        const T predicted_u = T(fx_) * point_camera[0] * inv_z + T(cx_);
        const T predicted_v = T(fy_) * point_camera[1] * inv_z + T(cy_);
        const T predicted_ur = predicted_u - T(bf_) * inv_z;
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        residuals[2] = T(sqrt_info_) * (predicted_ur - T(observed_ur_));
        return true;
    }

    /**
     * @brief Factory: residual 3, parameter block 6.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double observed_ur, double xw, double yw,
                                       double zw, double fx, double fy,
                                       double cx, double cy, double bf,
                                       double sqrt_info) {
        return new ceres::AutoDiffCostFunction<StereoPoseOnlyCostFunctor, 3, 6>(
            new StereoPoseOnlyCostFunctor(observed_u, observed_v, observed_ur,
                                          xw, yw, zw, fx, fy, cx, cy, bf,
                                          sqrt_info));
    }

    double observed_u_;
    double observed_v_;
    double observed_ur_;
    double xw_;
    double yw_;
    double zw_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double bf_;
    double sqrt_info_;
};

/**
 * @struct autonomy::localization::atlas::backend::StereoPoseOnlyFromTwbCostFunctor
 * @brief Stereo pose-only cost (\(T_{wb}\)). Residual \((u,v,u_r)\); right
 *        disparity uses \(bf/Z\).
 *
 * Residual dimension: 3. Parameter block: pose_wb(6).
 */
struct StereoPoseOnlyFromTwbCostFunctor {
    /**
     * @brief Construct stereo pose-only edge from \(T_{wb}\).
     * @param observed_u Left observed u.
     * @param observed_v Left observed v.
     * @param observed_ur Right observed u.
     * @param xw World point x.
     * @param yw World point y.
     * @param zw World point z.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param bf Baseline times focal length.
     * @param sqrt_info Square-root information weight.
     * @param R_cb Rotation \(R_{cb}\).
     * @param t_cb Translation \(t_{cb}\).
     */
    StereoPoseOnlyFromTwbCostFunctor(double observed_u, double observed_v,
                                     double observed_ur, double xw, double yw,
                                     double zw, double fx, double fy, double cx,
                                     double cy, double bf, double sqrt_info,
                                     const Mat33& R_cb, const Vec3& t_cb)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          observed_ur_(observed_ur),
          xw_(xw),
          yw_(yw),
          zw_(zw),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          bf_(bf),
          sqrt_info_(sqrt_info),
          R_cb_(R_cb),
          t_cb_(t_cb) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_wb \(T_{wb}\) parameter block.
     * @param[out] residuals Length-3 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_wb, T* residuals) const {
        T Rwb[9];
        ceres::AngleAxisToRotationMatrix(pose_wb, Rwb);
        const T dx = -Rwb[0] * pose_wb[3] - Rwb[1] * pose_wb[4] -
                     Rwb[2] * pose_wb[5];
        const T dy = -Rwb[3] * pose_wb[3] - Rwb[4] * pose_wb[4] -
                     Rwb[5] * pose_wb[5];
        const T dz = -Rwb[6] * pose_wb[3] - Rwb[7] * pose_wb[4] -
                     Rwb[8] * pose_wb[5];
        Eigen::Matrix<T, 3, 3> Rbw;
        Rbw << Rwb[0], Rwb[3], Rwb[6], Rwb[1], Rwb[4], Rwb[7], Rwb[2], Rwb[5],
            Rwb[8];
        const Eigen::Matrix<T, 3, 3> Rcw = R_cb_.cast<T>() * Rbw;
        const Eigen::Matrix<T, 3, 1> tcw =
            R_cb_.cast<T>() * Eigen::Matrix<T, 3, 1>(dx, dy, dz) +
            t_cb_.cast<T>();
        const Eigen::Matrix<T, 3, 1> pc =
            Rcw * Eigen::Matrix<T, 3, 1>(T(xw_), T(yw_), T(zw_)) + tcw;
        if (pc[2] <= T(1e-6)) {
            residuals[0] = residuals[1] = residuals[2] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / pc[2];
        const T pu = T(fx_) * pc[0] * inv_z + T(cx_);
        const T pv = T(fy_) * pc[1] * inv_z + T(cy_);
        residuals[0] = T(sqrt_info_) * (pu - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (pv - T(observed_v_));
        residuals[2] = T(sqrt_info_) * (pu - T(bf_) * inv_z - T(observed_ur_));
        return true;
    }

    /**
     * @brief Factory: residual 3, parameter block 6.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double observed_ur, double xw, double yw,
                                       double zw, double fx, double fy,
                                       double cx, double cy, double bf,
                                       double sqrt_info, const Mat33& R_cb,
                                       const Vec3& t_cb) {
        return new ceres::AutoDiffCostFunction<StereoPoseOnlyFromTwbCostFunctor,
                                               3, 6>(
            new StereoPoseOnlyFromTwbCostFunctor(
                observed_u, observed_v, observed_ur, xw, yw, zw, fx, fy, cx, cy,
                bf, sqrt_info, R_cb, t_cb));
    }

    double observed_u_;
    double observed_v_;
    double observed_ur_;
    double xw_;
    double yw_;
    double zw_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double bf_;
    double sqrt_info_;
    Mat33 R_cb_;
    Vec3 t_cb_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoReprojectionCostFunctor
 * @brief Monocular reprojection optimizing both pose and map point (LocalBA edge).
 *
 * Residual dimension: 2. Parameter blocks: pose(6) = \(T_{cw}\), point(3) =
 * world point.
 */
struct MonoReprojectionCostFunctor {
    /**
     * @brief Construct with ReprojCam.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param cam Camera intrinsics.
     * @param sqrt_info Square-root information weight.
     */
    MonoReprojectionCostFunctor(double observed_u, double observed_v,
                                const ReprojCam& cam, double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          cam_(cam),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     */
    MonoReprojectionCostFunctor(double observed_u, double observed_v, double fx,
                                double fy, double cx, double cy,
                                double sqrt_info)
        : MonoReprojectionCostFunctor(observed_u, observed_v,
                                      ReprojCam::Pinhole(fx, fy, cx, cy),
                                      sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose \(T_{cw}\) parameter block.
     * @param point World point parameter block.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose, const T* const point,
                    T* residuals) const {
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info) {
        return Create(observed_u, observed_v, ReprojCam::Pinhole(fx, fy, cx, cy),
                      sqrt_info);
    }

    /**
     * @brief Factory with ReprojCam: residual 2; blocks 6, 3.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       const ReprojCam& cam, double sqrt_info) {
        return new ceres::AutoDiffCostFunction<MonoReprojectionCostFunctor, 2,
                                               6, 3>(
            new MonoReprojectionCostFunctor(observed_u, observed_v, cam,
                                            sqrt_info));
    }

    double observed_u_;
    double observed_v_;
    ReprojCam cam_;
    double sqrt_info_;
};

/**
 * @struct autonomy::localization::atlas::backend::StereoReprojectionCostFunctor
 * @brief Stereo BA edge on \(T_{cw}\) and a map point (ORB EdgeStereo).
 *
 * Residual \((u,v,u_r)\), right disparity \(bf/Z\). Blocks: pose(6), point(3).
 */
struct StereoReprojectionCostFunctor {
    StereoReprojectionCostFunctor(double observed_u, double observed_v,
                                  double observed_ur, double fx, double fy,
                                  double cx, double cy, double bf,
                                  double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          observed_ur_(observed_ur),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          bf_(bf),
          sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const pose, const T* const point,
                    T* residuals) const {
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];
        if (point_camera[2] <= T(1e-6)) {
            residuals[0] = residuals[1] = residuals[2] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / point_camera[2];
        const T pu = T(fx_) * point_camera[0] * inv_z + T(cx_);
        const T pv = T(fy_) * point_camera[1] * inv_z + T(cy_);
        residuals[0] = T(sqrt_info_) * (pu - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (pv - T(observed_v_));
        residuals[2] =
            T(sqrt_info_) * (pu - T(bf_) * inv_z - T(observed_ur_));
        return true;
    }

    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double observed_ur, double fx, double fy,
                                       double cx, double cy, double bf,
                                       double sqrt_info) {
        return new ceres::AutoDiffCostFunction<StereoReprojectionCostFunctor, 3,
                                               6, 3>(
            new StereoReprojectionCostFunctor(observed_u, observed_v,
                                              observed_ur, fx, fy, cx, cy, bf,
                                              sqrt_info));
    }

    double observed_u_;
    double observed_v_;
    double observed_ur_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double bf_;
    double sqrt_info_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoReprojectionToBodyCostFunctor
 * @brief LocalBA right-camera edge (ORB EdgeSE3ProjectXYZToBody): optimizes
 *        pose and point.
 *
 * Residual dimension: 2. Parameter blocks: pose(6), point(3). \(T_{rl}\) is
 * fixed.
 */
struct MonoReprojectionToBodyCostFunctor {
    /**
     * @brief Construct with ReprojCam and left-to-right extrinsics.
     * @param observed_u Observed pixel u on the right camera.
     * @param observed_v Observed pixel v on the right camera.
     * @param cam Right-camera intrinsics.
     * @param sqrt_info Square-root information weight.
     * @param R_rl Rotation \(R_{rl}\).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoReprojectionToBodyCostFunctor(double observed_u, double observed_v,
                                      const ReprojCam& cam, double sqrt_info,
                                      const Mat33& R_rl, const Vec3& t_rl)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          cam_(cam),
          sqrt_info_(sqrt_info),
          R_rl_(R_rl),
          t_rl_(t_rl) {}

    /**
     * @brief Construct with pinhole intrinsics.
     * @param observed_u Observed pixel u.
     * @param observed_v Observed pixel v.
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     * @param R_rl Rotation \(R_{rl}\).
     * @param t_rl Translation \(t_{rl}\).
     */
    MonoReprojectionToBodyCostFunctor(double observed_u, double observed_v,
                                      double fx, double fy, double cx,
                                      double cy, double sqrt_info,
                                      const Mat33& R_rl, const Vec3& t_rl)
        : MonoReprojectionToBodyCostFunctor(
              observed_u, observed_v, ReprojCam::Pinhole(fx, fy, cx, cy),
              sqrt_info, R_rl, t_rl) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose Left-camera \(T_{cw}\) parameter block.
     * @param point World point parameter block.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose, const T* const point,
                    T* residuals) const {
        T point_left[3];
        ceres::AngleAxisRotatePoint(pose, point, point_left);
        point_left[0] += pose[3];
        point_left[1] += pose[4];
        point_left[2] += pose[5];

        T point_camera[3];
        point_camera[0] = T(R_rl_(0, 0)) * point_left[0] +
                          T(R_rl_(0, 1)) * point_left[1] +
                          T(R_rl_(0, 2)) * point_left[2] + T(t_rl_.x());
        point_camera[1] = T(R_rl_(1, 0)) * point_left[0] +
                          T(R_rl_(1, 1)) * point_left[1] +
                          T(R_rl_(1, 2)) * point_left[2] + T(t_rl_.y());
        point_camera[2] = T(R_rl_(2, 0)) * point_left[0] +
                          T(R_rl_(2, 1)) * point_left[1] +
                          T(R_rl_(2, 2)) * point_left[2] + T(t_rl_.z());
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory with pinhole intrinsics.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return Create(observed_u, observed_v,
                      ReprojCam::Pinhole(fx, fy, cx, cy), sqrt_info, R_rl,
                      t_rl);
    }

    /**
     * @brief Factory with ReprojCam: residual 2; blocks 6, 3.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       const ReprojCam& cam, double sqrt_info,
                                       const Mat33& R_rl, const Vec3& t_rl) {
        return new ceres::AutoDiffCostFunction<MonoReprojectionToBodyCostFunctor,
                                               2, 6, 3>(
            new MonoReprojectionToBodyCostFunctor(observed_u, observed_v, cam,
                                                  sqrt_info, R_rl, t_rl));
    }

    double observed_u_;
    double observed_v_;
    ReprojCam cam_;
    double sqrt_info_;
    Mat33 R_rl_;
    Vec3 t_rl_;
};

/**
 * @struct autonomy::localization::atlas::backend::MonoReprojectionWorldBodyCostFunctor
 * @brief Local/full inertial BA reprojection with pose \(T_{wb}\) and a map point.
 *
 * \(T_{cw}=T_{cb} T_{wb}^{-1}\). Residual 2. Blocks: pose_wb(6), point(3).
 */
struct MonoReprojectionWorldBodyCostFunctor {
    MonoReprojectionWorldBodyCostFunctor(double observed_u, double observed_v,
                                         const ReprojCam& cam, double sqrt_info,
                                         const Mat33& R_cb, const Vec3& t_cb)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          cam_(cam),
          sqrt_info_(sqrt_info),
          R_cb_(R_cb),
          t_cb_(t_cb) {}

    template <typename T>
    bool operator()(const T* const pose_wb, const T* const point,
                    T* residuals) const {
        T Rwb[9];
        ceres::AngleAxisToRotationMatrix(pose_wb, Rwb);
        const T dx = -Rwb[0] * pose_wb[3] - Rwb[1] * pose_wb[4] -
                     Rwb[2] * pose_wb[5];
        const T dy = -Rwb[3] * pose_wb[3] - Rwb[4] * pose_wb[4] -
                     Rwb[5] * pose_wb[5];
        const T dz = -Rwb[6] * pose_wb[3] - Rwb[7] * pose_wb[4] -
                     Rwb[8] * pose_wb[5];
        Eigen::Matrix<T, 3, 3> Rbw;
        Rbw << Rwb[0], Rwb[3], Rwb[6], Rwb[1], Rwb[4], Rwb[7], Rwb[2], Rwb[5],
            Rwb[8];
        const Eigen::Matrix<T, 3, 3> Rcw = R_cb_.cast<T>() * Rbw;
        const Eigen::Matrix<T, 3, 1> tcw =
            R_cb_.cast<T>() * Eigen::Matrix<T, 3, 1>(dx, dy, dz) +
            t_cb_.cast<T>();
        const Eigen::Matrix<T, 3, 1> pw(point[0], point[1], point[2]);
        const Eigen::Matrix<T, 3, 1> pc = Rcw * pw + tcw;
        T point_camera[3] = {pc[0], pc[1], pc[2]};
        T predicted_u;
        T predicted_v;
        if (!ProjectPixel(point_camera, cam_, &predicted_u, &predicted_v)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        residuals[0] = T(sqrt_info_) * (predicted_u - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (predicted_v - T(observed_v_));
        return true;
    }

    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       const ReprojCam& cam, double sqrt_info,
                                       const Mat33& R_cb, const Vec3& t_cb) {
        return new ceres::AutoDiffCostFunction<
            MonoReprojectionWorldBodyCostFunctor, 2, 6, 3>(
            new MonoReprojectionWorldBodyCostFunctor(observed_u, observed_v,
                                                     cam, sqrt_info, R_cb,
                                                     t_cb));
    }

    double observed_u_;
    double observed_v_;
    ReprojCam cam_;
    double sqrt_info_;
    Mat33 R_cb_;
    Vec3 t_cb_;
};

/**
 * @struct autonomy::localization::atlas::backend::StereoReprojectionWorldBodyCostFunctor
 * @brief Inertial BA stereo edge (ORB EdgeStereo) on \(T_{wb}\) and a map point.
 *
 * Residual \((u,v,u_r)\); right disparity is \(bf/Z\). Blocks: pose_wb(6), point(3).
 */
struct StereoReprojectionWorldBodyCostFunctor {
    StereoReprojectionWorldBodyCostFunctor(double observed_u, double observed_v,
                                           double observed_ur, double fx,
                                           double fy, double cx, double cy,
                                           double bf, double sqrt_info,
                                           const Mat33& R_cb, const Vec3& t_cb)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          observed_ur_(observed_ur),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          bf_(bf),
          sqrt_info_(sqrt_info),
          R_cb_(R_cb),
          t_cb_(t_cb) {}

    template <typename T>
    bool operator()(const T* const pose_wb, const T* const point,
                    T* residuals) const {
        T Rwb[9];
        ceres::AngleAxisToRotationMatrix(pose_wb, Rwb);
        const T dx = -Rwb[0] * pose_wb[3] - Rwb[1] * pose_wb[4] -
                     Rwb[2] * pose_wb[5];
        const T dy = -Rwb[3] * pose_wb[3] - Rwb[4] * pose_wb[4] -
                     Rwb[5] * pose_wb[5];
        const T dz = -Rwb[6] * pose_wb[3] - Rwb[7] * pose_wb[4] -
                     Rwb[8] * pose_wb[5];
        Eigen::Matrix<T, 3, 3> Rbw;
        Rbw << Rwb[0], Rwb[3], Rwb[6], Rwb[1], Rwb[4], Rwb[7], Rwb[2], Rwb[5],
            Rwb[8];
        const Eigen::Matrix<T, 3, 3> Rcw = R_cb_.cast<T>() * Rbw;
        const Eigen::Matrix<T, 3, 1> tcw =
            R_cb_.cast<T>() * Eigen::Matrix<T, 3, 1>(dx, dy, dz) +
            t_cb_.cast<T>();
        const Eigen::Matrix<T, 3, 1> pc =
            Rcw * Eigen::Matrix<T, 3, 1>(point[0], point[1], point[2]) + tcw;
        if (pc[2] <= T(1e-6)) {
            residuals[0] = residuals[1] = residuals[2] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / pc[2];
        const T pu = T(fx_) * pc[0] * inv_z + T(cx_);
        const T pv = T(fy_) * pc[1] * inv_z + T(cy_);
        residuals[0] = T(sqrt_info_) * (pu - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (pv - T(observed_v_));
        residuals[2] = T(sqrt_info_) * (pu - T(bf_) * inv_z - T(observed_ur_));
        return true;
    }

    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double observed_ur, double fx, double fy,
                                       double cx, double cy, double bf,
                                       double sqrt_info, const Mat33& R_cb,
                                       const Vec3& t_cb) {
        return new ceres::AutoDiffCostFunction<
            StereoReprojectionWorldBodyCostFunctor, 3, 6, 3>(
            new StereoReprojectionWorldBodyCostFunctor(
                observed_u, observed_v, observed_ur, fx, fy, cx, cy, bf,
                sqrt_info, R_cb, t_cb));
    }

    double observed_u_;
    double observed_v_;
    double observed_ur_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double bf_;
    double sqrt_info_;
    Mat33 R_cb_;
    Vec3 t_cb_;
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_REPROJECTION_COST_FUNCTION_HPP_
