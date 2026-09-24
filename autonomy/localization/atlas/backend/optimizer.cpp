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
 * @file optimizer.cpp
 * @brief Ceres implementations for Optimizer (PoseOptimization / LocalBA / Essential Graph, etc.).
 */

#include "autonomy/localization/atlas/backend/optimizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Eigenvalues"

#include "ceres/ceres.h"

#include "autonomy/localization/atlas/backend/cost_functions/cost_helpers.hpp"
#include "autonomy/localization/atlas/backend/cost_functions/imu_cost_function.hpp"
#include "autonomy/localization/atlas/backend/cost_functions/pose_prior_cost_function.hpp"
#include "autonomy/localization/atlas/backend/cost_functions/relative_pose_cost_function.hpp"
#include "autonomy/localization/atlas/backend/cost_functions/reprojection_cost_function.hpp"
#include "autonomy/localization/atlas/backend/cost_functions/sim3_cost_function.hpp"
#include "autonomy/localization/atlas/sensor/imu/pose.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {
namespace {

void Se3ToAngleAxisTranslation(const SE3& pose_cw, double* pose6) {
    const Eigen::AngleAxisd aa(pose_cw.rotation());
    Eigen::Map<Eigen::Vector3d> angle_axis(pose6);
    angle_axis = aa.axis() * aa.angle();
    Eigen::Map<Eigen::Vector3d> translation(pose6 + 3);
    translation = pose_cw.translation();
}

SE3 AngleAxisTranslationToSe3(const double* pose6) {
    const Eigen::Vector3d aa_vec = Eigen::Map<const Eigen::Vector3d>(pose6);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const double angle = aa_vec.norm();
    if (angle > 1e-12) {
        rotation =
            Eigen::AngleAxisd(angle, aa_vec / angle).toRotationMatrix();
    }
    SE3 pose = SE3Identity();
    pose.linear() = rotation;
    pose.translation() = Eigen::Map<const Eigen::Vector3d>(pose6 + 3);
    return pose;
}

double InvLevelSigma2KeyFrame(const KeyFrame& keyframe, int octave);

/** One visual residual in the joint inertial problem, for χ² rejection. */
struct InertialVisualEdge {
    ceres::ResidualBlockId block = nullptr;
    std::shared_ptr<KeyFrame> keyframe;
    std::shared_ptr<MapPoint> point;
    bool stereo = false;
};

/**
 * @brief Add map-point reprojections on \(T_{wb}\) into the same problem as IMU edges.
 *
 * Left observations are monocular (2) or rectified stereo \((u,v,u_r)\) (3).
 * Fisheye right-camera indices use \(T_{cb,r}=T_{rl}T_{cb}\).
 * When `edges` is non-null, each block is recorded for a later χ² check.
 */
void AddInertialVisualEdges(
    ceres::Problem* problem,
    const std::vector<std::shared_ptr<KeyFrame>>& keyframes,
    std::unordered_map<KeyFrame*, std::array<double, 6>>* pose_params,
    std::unordered_map<MapPoint*, std::array<double, 3>>* point_params,
    std::vector<InertialVisualEdge>* edges) {
    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;
    std::unordered_set<MapPoint*> seen;
    std::vector<std::shared_ptr<MapPoint>> points;
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        for (const auto& mp : kf->GetMapPoints()) {
            if (!mp || mp->isBad() || seen.count(mp.get())) {
                continue;
            }
            seen.insert(mp.get());
            points.push_back(mp);
        }
    }
    for (const auto& mp : points) {
        const Vec3 pw = mp->GetWorldPos();
        (*point_params)[mp.get()] = {pw.x(), pw.y(), pw.z()};
        problem->AddParameterBlock((*point_params)[mp.get()].data(), 3);
    }
    for (const auto& mp : points) {
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || pose_params->count(kf.get()) == 0) {
                continue;
            }
            if (index < 0 || index >= kf->TotalFeatures()) {
                continue;
            }
            const cv::KeyPoint kp = kf->GetKeyPoint(index);
            const double sqrt_info =
                std::sqrt(InvLevelSigma2KeyFrame(*kf, kp.octave));
            const auto calib = sensor::imu::DefaultCalibOr(kf->imu_calib);
            const Mat33 R_cb = calib.T_camera_body.rotation();
            const Vec3 t_cb = calib.T_camera_body.translation();
            ceres::CostFunction* cost = nullptr;
            double huber = std::sqrt(kChi2Mono);
            bool stereo = false;
            if (kf->HasDualCameraIndex() && index >= kf->num_left &&
                kf->camera2) {
                const SE3 Trl = kf->T_c1_c2.inverse();
                const Mat33 R_cb_right = Trl.rotation() * R_cb;
                const Vec3 t_cb_right =
                    Trl.rotation() * t_cb + Trl.translation();
                cost = MonoReprojectionWorldBodyCostFunctor::Create(
                    kp.pt.x, kp.pt.y,
                    ReprojCam::FromCamera(kf->camera2.get(), kf->fx, kf->fy,
                                          kf->cx, kf->cy),
                    sqrt_info, R_cb_right, t_cb_right);
            } else {
                const auto& right_u = kf->GetRightCoordinates();
                if (index < static_cast<int>(right_u.size()) &&
                    right_u[static_cast<size_t>(index)] >= 0.f &&
                    kf->baseline_times_fx > 1e-6f) {
                    stereo = true;
                    huber = std::sqrt(kChi2Stereo);
                    cost = StereoReprojectionWorldBodyCostFunctor::Create(
                        kp.pt.x, kp.pt.y,
                        right_u[static_cast<size_t>(index)], kf->fx, kf->fy,
                        kf->cx, kf->cy, kf->baseline_times_fx, sqrt_info, R_cb,
                        t_cb);
                } else {
                    cost = MonoReprojectionWorldBodyCostFunctor::Create(
                        kp.pt.x, kp.pt.y,
                        ReprojCam::FromCamera(kf->camera.get(), kf->fx, kf->fy,
                                              kf->cx, kf->cy),
                        sqrt_info, R_cb, t_cb);
                }
            }
            const ceres::ResidualBlockId block = problem->AddResidualBlock(
                cost, new ceres::HuberLoss(huber),
                (*pose_params)[kf.get()].data(),
                (*point_params)[mp.get()].data());
            if (edges != nullptr) {
                edges->push_back(InertialVisualEdge{block, kf, mp, stereo});
            }
        }
    }
}

double InvLevelSigma2(const tracking::Frame& frame, int octave) {
    if (octave >= 0 &&
        octave < static_cast<int>(frame.inverse_level_sigma2.size())) {
        return frame.inverse_level_sigma2[octave];
    }
    return 1.0;
}

double InvLevelSigma2KeyFrame(const KeyFrame& keyframe, int octave) {
    if (octave >= 0 &&
        octave < static_cast<int>(keyframe.inverse_level_sigma2.size())) {
        return keyframe.inverse_level_sigma2[octave];
    }
    return 1.0;
}

/** Mono, rectified stereo \((u,v,u_r)\), or fisheye right edge on \(T_{cw}\). */
void AddTcwReprojection(ceres::Problem* problem,
                        const std::shared_ptr<KeyFrame>& kf, int index,
                        double* pose, double* point) {
    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;
    const cv::KeyPoint kp = kf->GetKeyPoint(index);
    const double sqrt_info = std::sqrt(InvLevelSigma2KeyFrame(*kf, kp.octave));
    ceres::CostFunction* cost = nullptr;
    double huber = std::sqrt(kChi2Mono);
    if (kf->HasDualCameraIndex() && index >= kf->num_left && kf->camera2) {
        const SE3 Trl = kf->T_c1_c2.inverse();
        cost = MonoReprojectionToBodyCostFunctor::Create(
            kp.pt.x, kp.pt.y,
            ReprojCam::FromCamera(kf->camera2.get(), kf->fx, kf->fy, kf->cx,
                                  kf->cy),
            sqrt_info, Trl.linear(), Trl.translation());
    } else {
        const auto& right_u = kf->GetRightCoordinates();
        if (index < static_cast<int>(right_u.size()) &&
            right_u[static_cast<size_t>(index)] >= 0.f &&
            kf->baseline_times_fx > 1e-6f) {
            huber = std::sqrt(kChi2Stereo);
            cost = StereoReprojectionCostFunctor::Create(
                kp.pt.x, kp.pt.y, right_u[static_cast<size_t>(index)], kf->fx,
                kf->fy, kf->cx, kf->cy, kf->baseline_times_fx, sqrt_info);
        } else {
            cost = MonoReprojectionCostFunctor::Create(
                kp.pt.x, kp.pt.y,
                ReprojCam::FromCamera(kf->camera.get(), kf->fx, kf->fy, kf->cx,
                                      kf->cy),
                sqrt_info);
        }
    }
    problem->AddResidualBlock(cost, new ceres::HuberLoss(huber), pose, point);
}

/** Drop observations whose optimized reprojection \(\chi^2\) exceeds ORB thresholds. */
void RejectReprojectionOutliers(
    const std::unordered_map<KeyFrame*, std::array<double, 6>>& pose_params,
    const std::unordered_map<MapPoint*, std::array<double, 3>>& point_params) {
    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;
    for (const auto& [mp, p] : point_params) {
        if (!mp || mp->isBad()) {
            continue;
        }
        const Vec3 pw(p[0], p[1], p[2]);
        std::vector<std::pair<std::shared_ptr<KeyFrame>, int>> drop;
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || index < 0 ||
                index >= kf->TotalFeatures()) {
                continue;
            }
            const auto pose_it = pose_params.find(kf.get());
            if (pose_it == pose_params.end()) {
                continue;
            }
            const SE3 Tcw = AngleAxisTranslationToSe3(pose_it->second.data());
            Vec3 pc = Tcw * pw;
            const bool right_body =
                kf->HasDualCameraIndex() && index >= kf->num_left;
            if (right_body) {
                pc = kf->T_c1_c2.inverse() * pc;
            }
            const cv::KeyPoint kp = kf->GetKeyPoint(index);
            const double sqrt_info =
                std::sqrt(InvLevelSigma2KeyFrame(*kf, kp.octave));
            const sensor::GeometricCamera* geo =
                right_body ? kf->camera2.get() : kf->camera.get();
            double pu = 0.0;
            double pv = 0.0;
            if (geo) {
                const Vec2 uv = geo->Project(pc);
                pu = uv.x();
                pv = uv.y();
            } else if (pc.z() > 1e-6) {
                const double inv_z = 1.0 / pc.z();
                pu = kf->fx * pc.x() * inv_z + kf->cx;
                pv = kf->fy * pc.y() * inv_z + kf->cy;
            }
            const auto& right_u = kf->GetRightCoordinates();
            const bool stereo = !right_body &&
                                index < static_cast<int>(right_u.size()) &&
                                right_u[static_cast<size_t>(index)] >= 0.f &&
                                kf->baseline_times_fx > 1e-6f;
            const bool close = mp->track_depth < 10.f;
            bool outlier = pc.z() <= 1e-6;
            if (!outlier && stereo) {
                const double inv_z = 1.0 / pc.z();
                const double pur = pu - kf->baseline_times_fx * inv_z;
                const double eu = (pu - kp.pt.x) * sqrt_info;
                const double ev = (pv - kp.pt.y) * sqrt_info;
                const double er =
                    (pur - right_u[static_cast<size_t>(index)]) * sqrt_info;
                outlier = (eu * eu + ev * ev + er * er) > kChi2Stereo;
            } else if (!outlier) {
                const double eu = (pu - kp.pt.x) * sqrt_info;
                const double ev = (pv - kp.pt.y) * sqrt_info;
                const double thr = close ? 1.5 * kChi2Mono : kChi2Mono;
                outlier = (eu * eu + ev * ev) > thr;
            }
            if (outlier) {
                drop.emplace_back(kf, index);
            }
        }
        for (const auto& [kf, index] : drop) {
            kf->EraseMapPointMatch(index);
            mp->EraseObservation(kf);
        }
    }
}

}  // namespace

int Optimizer::PoseOptimization(tracking::Frame* frame) {
    if (frame == nullptr || !frame->has_pose()) {
        return 0;
    }

    struct Observation {
        int index = -1;
        bool stereo = false;
        bool right_body = false;  // fisheye dual right (ToBody)
        double u = 0.0;
        double v = 0.0;
        double ur = 0.0;
        double xw = 0.0;
        double yw = 0.0;
        double zw = 0.0;
        double sqrt_info = 1.0;
        ReprojCam cam;
        bool outlier = false;
    };

    const bool dual = frame->HasDualCameraIndex();
    const SE3 Trl = dual ? frame->T_c1_c2.inverse() : SE3Identity();
    const Mat33 R_rl = Trl.linear();
    const Vec3 t_rl = Trl.translation();

    const int n_feat = dual ? frame->TotalFeatures() : frame->num_keypoints;
    if (static_cast<int>(frame->outliers.size()) < n_feat) {
        frame->outliers.resize(static_cast<size_t>(n_feat), false);
    }

    std::vector<Observation> observations;
    observations.reserve(static_cast<size_t>(n_feat));
    for (int i = 0; i < n_feat; ++i) {
        if (i >= static_cast<int>(frame->map_points.size())) {
            break;
        }
        const auto& map_point = frame->map_points[static_cast<size_t>(i)];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        Observation obs;
        obs.index = i;
        const Vec3 pw = map_point->GetWorldPos();
        obs.xw = pw.x();
        obs.yw = pw.y();
        obs.zw = pw.z();
        const cv::KeyPoint kp = frame->GetKeyPoint(i);
        obs.u = kp.pt.x;
        obs.v = kp.pt.y;
        obs.sqrt_info = std::sqrt(InvLevelSigma2(*frame, kp.octave));
        frame->outliers[static_cast<size_t>(i)] = false;

        if (dual) {
            if (i < frame->num_left) {
                obs.cam = ReprojCam::FromCamera(frame->camera.get(), frame->fx,
                                                frame->fy, frame->cx, frame->cy);
            } else {
                obs.right_body = true;
                obs.cam = ReprojCam::FromCamera(frame->camera2.get(), frame->fx,
                                                frame->fy, frame->cx, frame->cy);
            }
        } else {
            obs.cam = ReprojCam::FromCamera(frame->camera.get(), frame->fx,
                                            frame->fy, frame->cx, frame->cy);
            if (i < static_cast<int>(frame->right_coordinate.size()) &&
                frame->right_coordinate[static_cast<size_t>(i)] >= 0.f) {
                obs.stereo = true;
                obs.ur = frame->right_coordinate[static_cast<size_t>(i)];
            }
        }
        observations.push_back(obs);
    }

    if (observations.size() < 3) {
        return static_cast<int>(observations.size());
    }

    double pose6[6];
    Se3ToAngleAxisTranslation(frame->GetPose(), pose6);

    // ORB-SLAM3: 4 rounds with Huber + chi2 outlier rejection.
    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;

    for (int round = 0; round < 4; ++round) {
        ceres::Problem problem;
        problem.AddParameterBlock(pose6, 6);

        for (const auto& obs : observations) {
            if (obs.outlier) {
                continue;
            }
            ceres::LossFunction* loss = nullptr;
            ceres::CostFunction* cost = nullptr;
            if (obs.right_body) {
                loss = new ceres::HuberLoss(std::sqrt(kChi2Mono));
                cost = MonoPoseOnlyToBodyCostFunctor::Create(
                    obs.u, obs.v, obs.xw, obs.yw, obs.zw, obs.cam,
                    obs.sqrt_info, R_rl, t_rl);
            } else if (obs.stereo) {
                loss = new ceres::HuberLoss(std::sqrt(kChi2Stereo));
                cost = StereoPoseOnlyCostFunctor::Create(
                    obs.u, obs.v, obs.ur, obs.xw, obs.yw, obs.zw, frame->fx,
                    frame->fy, frame->cx, frame->cy, frame->baseline_times_fx,
                    obs.sqrt_info);
            } else {
                loss = new ceres::HuberLoss(std::sqrt(kChi2Mono));
                cost = MonoPoseOnlyCostFunctor::Create(
                    obs.u, obs.v, obs.xw, obs.yw, obs.zw, obs.cam,
                    obs.sqrt_info);
            }
            problem.AddResidualBlock(cost, loss, pose6);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        for (auto& obs : observations) {
            const SE3 Tcw = AngleAxisTranslationToSe3(pose6);
            Vec3 pc = Tcw * Vec3(obs.xw, obs.yw, obs.zw);
            if (obs.right_body) {
                pc = Trl * pc;
            }
            double pu = 0.0;
            double pv = 0.0;
            const sensor::GeometricCamera* geo =
                obs.right_body ? frame->camera2.get() : frame->camera.get();
            if (geo) {
                if (!geo->IsInValidRange(pc) && !obs.cam.kannala) {
                    obs.outlier = true;
                    continue;
                }
                const Vec2 uv = geo->Project(pc);
                pu = uv.x();
                pv = uv.y();
            } else {
                if (pc.z() <= 1e-6) {
                    obs.outlier = true;
                    continue;
                }
                const double inv_z = 1.0 / pc.z();
                pu = obs.cam.fx * pc.x() * inv_z + obs.cam.cx;
                pv = obs.cam.fy * pc.y() * inv_z + obs.cam.cy;
            }
            if (obs.stereo) {
                if (pc.z() <= 1e-6) {
                    obs.outlier = true;
                    continue;
                }
                const double inv_z = 1.0 / pc.z();
                const double pur = pu - frame->baseline_times_fx * inv_z;
                const double eu = (pu - obs.u) * obs.sqrt_info;
                const double ev = (pv - obs.v) * obs.sqrt_info;
                const double er = (pur - obs.ur) * obs.sqrt_info;
                obs.outlier = (eu * eu + ev * ev + er * er) > kChi2Stereo;
            } else {
                const double eu = (pu - obs.u) * obs.sqrt_info;
                const double ev = (pv - obs.v) * obs.sqrt_info;
                obs.outlier = (eu * eu + ev * ev) > kChi2Mono;
            }
        }
    }

    frame->SetPose(AngleAxisTranslationToSe3(pose6));
    int inliers = 0;
    for (const auto& obs : observations) {
        frame->outliers[static_cast<size_t>(obs.index)] = obs.outlier;
        if (!obs.outlier) {
            ++inliers;
        }
    }
    return inliers;
}

void Optimizer::LocalBundleAdjustment(
    const std::shared_ptr<KeyFrame>& keyframe, Map* map, bool* stop_flag) {
    if (!keyframe || !map) {
        return;
    }
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }

    std::vector<std::shared_ptr<KeyFrame>> local_keyframes =
        keyframe->GetBestCovisibilityKeyFrames(25);
    if (std::find(local_keyframes.begin(), local_keyframes.end(), keyframe) ==
        local_keyframes.end()) {
        local_keyframes.insert(local_keyframes.begin(), keyframe);
    }

    std::unordered_set<MapPoint*> local_point_set;
    std::vector<std::shared_ptr<MapPoint>> local_points;
    for (const auto& kf : local_keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        for (const auto& mp : kf->GetMapPoints()) {
            if (!mp || mp->isBad() || local_point_set.count(mp.get())) {
                continue;
            }
            local_point_set.insert(mp.get());
            local_points.push_back(mp);
        }
    }
    if (local_keyframes.size() < 2 || local_points.empty()) {
        return;
    }

    // Fixed keyframes: see local points but not in local window.
    std::unordered_set<KeyFrame*> local_kf_set;
    for (const auto& kf : local_keyframes) {
        if (kf) {
            local_kf_set.insert(kf.get());
        }
    }
    std::vector<std::shared_ptr<KeyFrame>> fixed_keyframes;
    std::unordered_set<KeyFrame*> fixed_set;
    for (const auto& mp : local_points) {
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            (void)index;
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || local_kf_set.count(kf.get()) ||
                fixed_set.count(kf.get())) {
                continue;
            }
            fixed_set.insert(kf.get());
            fixed_keyframes.push_back(kf);
        }
    }

    // Pose / point parameter blocks.
    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    std::unordered_map<MapPoint*, std::array<double, 3>> point_params;

    ceres::Problem problem;
    for (const auto& kf : local_keyframes) {
        if (!kf) {
            continue;
        }
        Se3ToAngleAxisTranslation(kf->GetPose(), pose_params[kf.get()].data());
        problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
        // Fix oldest / first local KF for gauge freedom.
        if (kf == local_keyframes.front()) {
            problem.SetParameterBlockConstant(pose_params[kf.get()].data());
        }
    }
    for (const auto& kf : fixed_keyframes) {
        if (!kf) {
            continue;
        }
        Se3ToAngleAxisTranslation(kf->GetPose(), pose_params[kf.get()].data());
        problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
        problem.SetParameterBlockConstant(pose_params[kf.get()].data());
    }
    for (const auto& mp : local_points) {
        const Vec3 pw = mp->GetWorldPos();
        point_params[mp.get()] = {pw.x(), pw.y(), pw.z()};
        problem.AddParameterBlock(point_params[mp.get()].data(), 3);
    }

    for (const auto& mp : local_points) {
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || pose_params.count(kf.get()) == 0) {
                continue;
            }
            if (index < 0 || index >= kf->TotalFeatures()) {
                continue;
            }
            AddTcwReprojection(&problem, kf, index,
                               pose_params[kf.get()].data(),
                               point_params[mp.get()].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    if (stop_flag != nullptr) {
        // Ceres has no force-stop flag identical to g2o; check before solve.
        if (*stop_flag) {
            return;
        }
    }
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }
    RejectReprojectionOutliers(pose_params, point_params);

    for (const auto& kf : local_keyframes) {
        if (!kf || kf == local_keyframes.front()) {
            // Front was fixed — still write for consistency.
        }
        if (!kf) {
            continue;
        }
        kf->SetPose(AngleAxisTranslationToSe3(pose_params[kf.get()].data()));
    }
    for (const auto& mp : local_points) {
        const auto& p = point_params[mp.get()];
        mp->SetWorldPos(Vec3(p[0], p[1], p[2]));
        mp->UpdateNormalAndDepth();
    }
}

void Optimizer::GlobalBundleAdjustment(Map* map, int iterations,
                                       bool* stop_flag, uint64_t gba_id) {
    if (!map) {
        return;
    }
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }
    const auto keyframes = map->GetAllKeyFrames();
    const auto map_points = map->GetAllMapPoints();
    if (keyframes.size() < 2 || map_points.empty()) {
        return;
    }

    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    std::unordered_map<MapPoint*, std::array<double, 3>> point_params;
    ceres::Problem problem;

    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        Se3ToAngleAxisTranslation(kf->GetPose(), pose_params[kf.get()].data());
        problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
    }
    // Fix first keyframe for gauge.
    if (!keyframes.empty() && pose_params.count(keyframes.front().get())) {
        problem.SetParameterBlockConstant(
            pose_params[keyframes.front().get()].data());
    }
    for (const auto& mp : map_points) {
        if (!mp || mp->isBad()) {
            continue;
        }
        const Vec3 pw = mp->GetWorldPos();
        point_params[mp.get()] = {pw.x(), pw.y(), pw.z()};
        problem.AddParameterBlock(point_params[mp.get()].data(), 3);
    }

    for (const auto& mp : map_points) {
        if (!mp || point_params.count(mp.get()) == 0) {
            continue;
        }
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || pose_params.count(kf.get()) == 0) {
                continue;
            }
            if (index < 0 || index >= kf->TotalFeatures()) {
                continue;
            }
            AddTcwReprojection(&problem, kf, index,
                               pose_params[kf.get()].data(),
                               point_params[mp.get()].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = std::max(1, iterations);
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }
    RejectReprojectionOutliers(pose_params, point_params);

    for (const auto& kf : keyframes) {
        if (!kf || pose_params.count(kf.get()) == 0) {
            continue;
        }
        const SE3 Tcw =
            AngleAxisTranslationToSe3(pose_params[kf.get()].data());
        if (gba_id == 0) {
            kf->SetPose(Tcw);
        } else {
            kf->pose_gba = Tcw;
            kf->gba_for_kf = gba_id;
        }
    }
    for (const auto& mp : map_points) {
        if (!mp || point_params.count(mp.get()) == 0) {
            continue;
        }
        const auto& p = point_params[mp.get()];
        const Vec3 pw(p[0], p[1], p[2]);
        if (gba_id == 0) {
            mp->SetWorldPos(pw);
            mp->UpdateNormalAndDepth();
        } else {
            mp->position_gba = pw;
            mp->gba_for_kf = gba_id;
        }
    }
}

void Optimizer::OptimizeEssentialGraph(
    Map* map, const std::shared_ptr<KeyFrame>& loop_keyframe,
    const std::shared_ptr<KeyFrame>& current_keyframe,
    const SE3& loop_relative_pose_cw,
    const std::unordered_map<KeyFrame*, SE3>* pose_before) {
    if (!map || !loop_keyframe || !current_keyframe) {
        return;
    }
    const auto keyframes = map->GetAllKeyFrames();
    if (keyframes.size() < 2) {
        return;
    }

    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    ceres::Problem problem;
    for (const auto& keyframe : keyframes) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        Se3ToAngleAxisTranslation(keyframe->GetPose(),
                                  pose_params[keyframe.get()].data());
        problem.AddParameterBlock(pose_params[keyframe.get()].data(), 6);
    }
    if (pose_params.count(loop_keyframe.get())) {
        problem.SetParameterBlockConstant(
            pose_params[loop_keyframe.get()].data());
    }

    auto AddRelativeEdge = [&](const std::shared_ptr<KeyFrame>& a,
                               const std::shared_ptr<KeyFrame>& b,
                               const SE3& T_ab, double weight) {
        if (!a || !b || pose_params.count(a.get()) == 0 ||
            pose_params.count(b.get()) == 0) {
            return;
        }
        problem.AddResidualBlock(
            RelativeSe3CostFunctor::Create(T_ab.rotation(), T_ab.translation(),
                                           weight),
            nullptr, pose_params[a.get()].data(), pose_params[b.get()].data());
    };

    auto PoseBefore = [&](const std::shared_ptr<KeyFrame>& kf) {
        if (pose_before != nullptr) {
            const auto it = pose_before->find(kf.get());
            if (it != pose_before->end()) {
                return it->second;
            }
        }
        return kf->GetPose();
    };
    std::set<std::pair<KeyFrame*, KeyFrame*>> linked;
    auto AddMeasured = [&](const std::shared_ptr<KeyFrame>& a,
                           const std::shared_ptr<KeyFrame>& b, double weight) {
        if (!a || !b || a.get() == b.get()) {
            return;
        }
        KeyFrame* lo = a.get() < b.get() ? a.get() : b.get();
        KeyFrame* hi = a.get() < b.get() ? b.get() : a.get();
        if (weight <= 1.0 && !linked.insert({lo, hi}).second) {
            return;
        }
        AddRelativeEdge(a, b, PoseBefore(a) * PoseBefore(b).inverse(), weight);
    };

    for (const auto& keyframe : keyframes) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        for (const auto& neighbor :
             keyframe->GetBestCovisibilityKeyFrames(15)) {
            if (!neighbor || neighbor->id <= keyframe->id) {
                continue;
            }
            AddMeasured(keyframe, neighbor, 1.0);
        }
        AddMeasured(keyframe, keyframe->GetParent(), 1.0);
        for (const auto& loop_kf : keyframe->GetLoopEdges()) {
            AddMeasured(keyframe, loop_kf, 1.0);
        }
    }
    AddRelativeEdge(current_keyframe, loop_keyframe, loop_relative_pose_cw,
                    10.0);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Correct map points by the motion of a reference observing keyframe.
    std::unordered_map<KeyFrame*, SE3> old_poses;
    for (const auto& keyframe : keyframes) {
        if (!keyframe || pose_params.count(keyframe.get()) == 0) {
            continue;
        }
        old_poses[keyframe.get()] = keyframe->GetPose();
    }
    for (const auto& keyframe : keyframes) {
        if (!keyframe || pose_params.count(keyframe.get()) == 0) {
            continue;
        }
        keyframe->SetPose(
            AngleAxisTranslationToSe3(pose_params[keyframe.get()].data()));
    }
    for (const auto& map_point : map->GetAllMapPoints()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        auto observations = map_point->GetObservations();
        if (observations.empty()) {
            continue;
        }
        auto ref = observations.begin()->first.lock();
        if (!ref || old_poses.count(ref.get()) == 0) {
            continue;
        }
        const SE3 T_old = old_poses[ref.get()];
        const SE3 T_new = ref->GetPose();
        const Vec3 Pw = map_point->GetWorldPos();
        const Vec3 Pc = T_old * Pw;
        map_point->SetWorldPos(T_new.inverse() * Pc);
        map_point->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2,
    const std::vector<std::shared_ptr<MapPoint>>& matches12, Sim3* sim3_12,
    float thr2, bool fix_scale, bool all_points) {
    if (!keyframe1 || !keyframe2 || sim3_12 == nullptr) {
        return 0;
    }
    const SE3 Tcw1 = keyframe1->GetPose();
    const SE3 Tcw2 = keyframe2->GetPose();
    const auto map_points1 = keyframe1->GetMapPoints();
    const auto& keys1 = keyframe1->GetKeyPoints();
    const auto& keys2 = keyframe2->GetKeyPoints();

    // Parameters: aa(3), t(3), log_scale(1).
    double sim3[7] = {0, 0, 0, 0, 0, 0, 0};
    {
        const Eigen::AngleAxisd aa(sim3_12->rotation);
        Eigen::Map<Eigen::Vector3d> rotation(sim3);
        rotation = aa.axis() * aa.angle();
        Eigen::Map<Eigen::Vector3d> translation(sim3 + 3);
        translation = sim3_12->translation;
        sim3[6] = std::log(std::max(1e-9, sim3_12->scale));
    }

    struct Obs {
        double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
        double x1 = 0, y1 = 0, z1 = 0;
        double x2 = 0, y2 = 0, z2 = 0;
        double sqrt_info = 1.0;
        bool has_inverse = false;
        bool valid = false;
    };
    std::vector<Obs> observations;
    observations.reserve(matches12.size());

    for (size_t i = 0; i < matches12.size(); ++i) {
        const auto& mp2 = matches12[i];
        if (!mp2 || mp2->isBad() || i >= keys1.size()) {
            continue;
        }
        const std::shared_ptr<MapPoint> mp1 =
            (i < map_points1.size()) ? map_points1[i] : nullptr;
        const bool has_mp1 = mp1 && !mp1->isBad();
        if (!has_mp1 && !all_points) {
            continue;
        }

        int idx2 = -1;
        for (const auto& [weak_kf, index] : mp2->GetObservations()) {
            auto kf = weak_kf.lock();
            if (kf && kf.get() == keyframe2.get()) {
                idx2 = index;
                break;
            }
        }
        if (idx2 < 0 && !all_points) {
            continue;
        }

        const Vec3 Xc2 = Tcw2 * mp2->GetWorldPos();
        if (Xc2.z() <= 0.0) {
            continue;
        }

        Obs obs;
        obs.u1 = keys1[i].pt.x;
        obs.v1 = keys1[i].pt.y;
        obs.x2 = Xc2.x();
        obs.y2 = Xc2.y();
        obs.z2 = Xc2.z();
        obs.sqrt_info = std::sqrt(
            InvLevelSigma2KeyFrame(*keyframe1, keys1[i].octave));

        if (idx2 >= 0 && static_cast<size_t>(idx2) < keys2.size()) {
            obs.u2 = keys2[static_cast<size_t>(idx2)].pt.x;
            obs.v2 = keys2[static_cast<size_t>(idx2)].pt.y;
        } else {
            const double invz = 1.0 / Xc2.z();
            obs.u2 = keyframe2->fx * Xc2.x() * invz + keyframe2->cx;
            obs.v2 = keyframe2->fy * Xc2.y() * invz + keyframe2->cy;
        }

        if (has_mp1) {
            const Vec3 Xc1 = Tcw1 * mp1->GetWorldPos();
            if (Xc1.z() <= 0.0) {
                if (!all_points) {
                    continue;
                }
            } else {
                obs.x1 = Xc1.x();
                obs.y1 = Xc1.y();
                obs.z1 = Xc1.z();
                obs.has_inverse = true;
            }
        }
        obs.valid = true;
        observations.push_back(obs);
    }
    if (observations.size() < 3) {
        return 0;
    }

    ceres::Problem problem;
    problem.AddParameterBlock(sim3, 7);
    if (fix_scale) {
        problem.SetParameterLowerBound(sim3, 6, 0.0);
        problem.SetParameterUpperBound(sim3, 6, 0.0);
    }

    const double huber = std::sqrt(static_cast<double>(thr2));
    for (const auto& obs : observations) {
        problem.AddResidualBlock(
            Sim3ProjectCostFunctor::Create(obs.u1, obs.v1, obs.x2, obs.y2,
                                           obs.z2, keyframe1->fx, keyframe1->fy,
                                           keyframe1->cx, keyframe1->cy,
                                           obs.sqrt_info),
            new ceres::HuberLoss(huber), sim3);
        if (obs.has_inverse) {
            problem.AddResidualBlock(
                InverseSim3ProjectCostFunctor::Create(
                    obs.u2, obs.v2, obs.x1, obs.y1, obs.z1, keyframe2->fx,
                    keyframe2->fy, keyframe2->cx, keyframe2->cy, obs.sqrt_info),
                new ceres::HuberLoss(huber), sim3);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    {
        const Eigen::Vector3d aa_vec = Eigen::Map<const Eigen::Vector3d>(sim3);
        const double angle = aa_vec.norm();
        if (angle > 1e-12) {
            sim3_12->rotation =
                Eigen::AngleAxisd(angle, aa_vec / angle).toRotationMatrix();
        } else {
            sim3_12->rotation = Mat33::Identity();
        }
        sim3_12->translation = Eigen::Map<const Eigen::Vector3d>(sim3 + 3);
        sim3_12->scale = fix_scale ? 1.0 : std::exp(sim3[6]);
    }

    // Count inliers by forward reprojection.
    int inliers = 0;
    for (const auto& obs : observations) {
        const Vec3 mapped =
            sim3_12->Map(Vec3(obs.x2, obs.y2, obs.z2));
        if (mapped.z() <= 1e-6) {
            continue;
        }
        const double inv_z = 1.0 / mapped.z();
        const double pu = keyframe1->fx * mapped.x() * inv_z + keyframe1->cx;
        const double pv = keyframe1->fy * mapped.y() * inv_z + keyframe1->cy;
        const double eu = (pu - obs.u1) * obs.sqrt_info;
        const double ev = (pv - obs.v1) * obs.sqrt_info;
        if (eu * eu + ev * ev < thr2) {
            ++inliers;
        }
    }
    return inliers;
}

void Optimizer::LocalInertialBA(const std::shared_ptr<KeyFrame>& keyframe,
                                Map* map, bool* stop_flag, bool large,
                                bool reinit) {
    if (!keyframe || !map) {
        return;
    }
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }

    // Temporal window: newest first. `large` uses 25 keyframes and 4 iterations
    // (ORB bLarge). The keyframe just before the window is the fixed anchor.
    const int kMaxOpt = large ? 25 : 10;
    constexpr int kMaxFixed = 200;
    std::vector<std::shared_ptr<KeyFrame>> optimizable;
    auto cursor = keyframe;
    for (int i = 0; i < kMaxOpt && cursor && !cursor->isBad(); ++i) {
        optimizable.push_back(cursor);
        cursor = cursor->previous_keyframe.lock();
    }
    std::shared_ptr<KeyFrame> fixed_anchor;
    if (cursor && !cursor->isBad()) {
        fixed_anchor = cursor;
    } else if (optimizable.size() > 1) {
        fixed_anchor = optimizable.back();
        optimizable.pop_back();
    }
    bool has_imu_edge = false;
    for (const auto& kf : optimizable) {
        if (kf && kf->imu_preintegrated) {
            has_imu_edge = true;
            break;
        }
    }
    if (!has_imu_edge || optimizable.empty()) {
        LocalBundleAdjustment(keyframe, map, stop_flag);
        return;
    }

    auto add_imu_state = [](ceres::Problem* problem, KeyFrame* kf,
                            std::unordered_map<KeyFrame*, std::array<double, 6>>*
                                pose_params,
                            std::unordered_map<KeyFrame*, std::array<double, 3>>*
                                vel_params,
                            std::unordered_map<KeyFrame*, std::array<double, 6>>*
                                bias_params,
                            bool fixed) {
        const SE3 Twb = kf->GetImuPose();
        Se3ToAngleAxisTranslation(Twb, (*pose_params)[kf].data());
        problem->AddParameterBlock((*pose_params)[kf].data(), 6);
        (*vel_params)[kf] = {kf->velocity_world.x(), kf->velocity_world.y(),
                             kf->velocity_world.z()};
        problem->AddParameterBlock((*vel_params)[kf].data(), 3);
        (*bias_params)[kf] = {
            kf->imu_bias.gyroscope.x(),     kf->imu_bias.gyroscope.y(),
            kf->imu_bias.gyroscope.z(),     kf->imu_bias.accelerometer.x(),
            kf->imu_bias.accelerometer.y(), kf->imu_bias.accelerometer.z()};
        problem->AddParameterBlock((*bias_params)[kf].data(), 6);
        if (fixed) {
            problem->SetParameterBlockConstant((*pose_params)[kf].data());
            problem->SetParameterBlockConstant((*vel_params)[kf].data());
            problem->SetParameterBlockConstant((*bias_params)[kf].data());
        }
    };

    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    std::unordered_map<KeyFrame*, std::array<double, 3>> vel_params;
    std::unordered_map<KeyFrame*, std::array<double, 6>> bias_params;
    ceres::Problem problem;
    for (const auto& kf : optimizable) {
        add_imu_state(&problem, kf.get(), &pose_params, &vel_params,
                      &bias_params, false);
    }
    if (fixed_anchor) {
        add_imu_state(&problem, fixed_anchor.get(), &pose_params, &vel_params,
                      &bias_params, true);
    }

    std::unordered_set<KeyFrame*> in_window;
    for (const auto& kf : optimizable) {
        in_window.insert(kf.get());
    }
    if (fixed_anchor) {
        in_window.insert(fixed_anchor.get());
    }
    std::vector<std::shared_ptr<MapPoint>> local_points;
    std::unordered_set<MapPoint*> seen_points;
    for (const auto& kf : optimizable) {
        for (const auto& mp : kf->GetMapPoints()) {
            if (!mp || mp->isBad() || seen_points.count(mp.get())) {
                continue;
            }
            seen_points.insert(mp.get());
            local_points.push_back(mp);
        }
    }
    int fixed_visual = 0;
    for (const auto& mp : local_points) {
        if (fixed_visual >= kMaxFixed) {
            break;
        }
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            (void)index;
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || in_window.count(kf.get())) {
                continue;
            }
            in_window.insert(kf.get());
            Se3ToAngleAxisTranslation(kf->GetImuPose(),
                                      pose_params[kf.get()].data());
            problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
            problem.SetParameterBlockConstant(pose_params[kf.get()].data());
            ++fixed_visual;
            break;
        }
    }

    const Vec3 gravity(0.0, 0.0, -sensor::imu::kGravity);
    constexpr double kBoundaryInfo = 1e-2;
    constexpr double kChi2Inertial = 16.92;
    for (const auto& kf_j : optimizable) {
        if (!kf_j || !kf_j->imu_preintegrated) {
            continue;
        }
        auto kf_i = kf_j->previous_keyframe.lock();
        if (!kf_i || bias_params.count(kf_i.get()) == 0 ||
            bias_params.count(kf_j.get()) == 0) {
            continue;
        }
        const bool boundary = fixed_anchor && kf_i.get() == fixed_anchor.get();
        const bool robust = boundary || reinit;
        problem.AddResidualBlock(
            ImuPreintegrationBiasCostFunctor::Create(
                *kf_j->imu_preintegrated, gravity,
                boundary ? kBoundaryInfo : 1.0),
            robust ? new ceres::HuberLoss(std::sqrt(kChi2Inertial)) : nullptr,
            pose_params[kf_i.get()].data(), pose_params[kf_j.get()].data(),
            vel_params[kf_i.get()].data(), vel_params[kf_j.get()].data(),
            bias_params[kf_i.get()].data());
        problem.AddResidualBlock(
            ImuBiasWalkCostFunctor::CreateFromPreintegrator(
                *kf_j->imu_preintegrated),
            nullptr, bias_params[kf_i.get()].data(),
            bias_params[kf_j.get()].data());
    }

    std::unordered_map<MapPoint*, std::array<double, 3>> point_params;
    std::vector<InertialVisualEdge> visual_edges;
    AddInertialVisualEdges(&problem, optimizable, &pose_params, &point_params,
                           &visual_edges);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = large ? 4 : 10;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    const bool diverged =
        !std::isfinite(summary.final_cost) ||
        (std::isfinite(summary.initial_cost) &&
         2.0 * summary.initial_cost < summary.final_cost);
    if (diverged && !large) {
        return;
    }

    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;
    for (const auto& edge : visual_edges) {
        if (edge.block == nullptr || !edge.keyframe || !edge.point ||
            edge.point->isBad()) {
            continue;
        }
        ceres::Problem::EvaluateOptions eval;
        eval.residual_blocks = {edge.block};
        eval.apply_loss_function = false;
        double cost = 0.0;
        if (!problem.Evaluate(eval, &cost, nullptr, nullptr, nullptr)) {
            continue;
        }
        const double chi2 = 2.0 * cost;
        const bool close = edge.point->track_depth < 10.f;
        const double threshold =
            edge.stereo ? kChi2Stereo : (close ? 1.5 * kChi2Mono : kChi2Mono);
        if (chi2 > threshold) {
            edge.keyframe->EraseMapPointMatch(edge.point);
            edge.point->EraseObservation(edge.keyframe);
        }
    }

    for (const auto& kf : optimizable) {
        if (!kf || pose_params.count(kf.get()) == 0) {
            continue;
        }
        kf->SetImuPose(AngleAxisTranslationToSe3(pose_params[kf.get()].data()));
        const auto& v = vel_params[kf.get()];
        kf->velocity_world = Vec3(v[0], v[1], v[2]);
        kf->has_velocity = true;
        const auto& b = bias_params[kf.get()];
        kf->imu_bias.gyroscope = Vec3(b[0], b[1], b[2]);
        kf->imu_bias.accelerometer = Vec3(b[3], b[4], b[5]);
        if (kf->imu_preintegrated) {
            kf->imu_preintegrated->SetNewBias(kf->imu_bias);
        }
    }
    for (const auto& [mp, p] : point_params) {
        if (!mp || mp->isBad()) {
            continue;
        }
        mp->SetWorldPos(Vec3(p[0], p[1], p[2]));
        mp->UpdateNormalAndDepth();
    }
}

void Optimizer::InertialOptimization(Map* map, Mat33* Rwg, double* scale,
                                     Vec3* bg, Vec3* ba, bool monocular,
                                     float prior_g, float prior_a,
                                     bool fix_gravity_scale,
                                     Eigen::MatrixXd* covariance) {
    if (!map || !Rwg || !scale || !bg || !ba) {
        return;
    }
    auto keyframes = map->GetAllKeyFrames();
    if (keyframes.size() < 10) {
        return;
    }
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    // Seed gravity direction + velocities (ORB-SLAM3 InitializeIMU pre-pass).
    Vec3 dir_g = Vec3::Zero();
    int count = 0;
    for (size_t i = 1; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        auto prev = kf->previous_keyframe.lock();
        if (!kf || kf->isBad() || !kf->imu_preintegrated || !prev ||
            prev->isBad()) {
            continue;
        }
        const Mat33 Rwb = prev->GetImuPose().rotation();
        dir_g -= Rwb * kf->imu_preintegrated->GetUpdatedDeltaVelocity();
        const double dt = std::max(1e-3, kf->imu_preintegrated->delta_t);
        const Vec3 vel =
            (kf->GetImuPose().translation() - prev->GetImuPose().translation()) /
            dt;
        kf->velocity_world = vel;
        kf->has_velocity = true;
        prev->velocity_world = vel;
        prev->has_velocity = true;
        ++count;
    }
    if (count == 0) {
        return;
    }
    if (dir_g.norm() > 1e-6 && !map->isImuInitialized()) {
        dir_g.normalize();
        const Vec3 gI(0.0, 0.0, -1.0);
        Vec3 v = gI.cross(dir_g);
        const double nv = v.norm();
        const double cosg = std::max(-1.0, std::min(1.0, gI.dot(dir_g)));
        const double ang = std::acos(cosg);
        if (nv > 1e-8) {
            const Vec3 axis = v / nv * ang;
            *Rwg =
                Eigen::AngleAxisd(axis.norm(), axis.normalized()).toRotationMatrix();
        } else {
            *Rwg = Mat33::Identity();
        }
    } else if (!map->isImuInitialized()) {
        *Rwg = Mat33::Identity();
    }

    // Shared bias + per-KF velocity + Rwg + scale (EdgeInertialGS).
    std::unordered_map<KeyFrame*, std::array<double, 3>> vel_params;
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        std::array<double, 3> v = {kf->velocity_world.x(),
                                   kf->velocity_world.y(),
                                   kf->velocity_world.z()};
        vel_params[kf.get()] = v;
    }
    std::array<double, 6> bias = {bg->x(), bg->y(), bg->z(),
                                  ba->x(), ba->y(), ba->z()};
    if (!map->isImuInitialized()) {
        bias = {0, 0, 0, 0, 0, 0};
    } else if (!keyframes.empty() && keyframes.front()) {
        const auto& b0 = keyframes.front()->imu_bias;
        bias = {b0.gyroscope.x(),     b0.gyroscope.y(),     b0.gyroscope.z(),
                b0.accelerometer.x(), b0.accelerometer.y(), b0.accelerometer.z()};
    }

    const Eigen::AngleAxisd aa_rwg(*Rwg);
    std::array<double, 3> rwg_aa = {
        aa_rwg.axis().x() * aa_rwg.angle(),
        aa_rwg.axis().y() * aa_rwg.angle(),
        aa_rwg.axis().z() * aa_rwg.angle()};
    double scale_param = (*scale > 1e-6) ? *scale : 1.0;

    ceres::Problem problem;
    problem.AddParameterBlock(bias.data(), 6);
    problem.AddParameterBlock(rwg_aa.data(), 3);
    problem.AddParameterBlock(&scale_param, 1);
    if (!monocular || fix_gravity_scale) {
        problem.SetParameterBlockConstant(&scale_param);
    }
    if (fix_gravity_scale) {
        problem.SetParameterBlockConstant(rwg_aa.data());
    }

    int edges = 0;
    for (const auto& kf_j : keyframes) {
        if (!kf_j || kf_j->isBad() || !kf_j->imu_preintegrated) {
            continue;
        }
        auto kf_i = kf_j->previous_keyframe.lock();
        if (!kf_i || kf_i->isBad() || !vel_params.count(kf_i.get()) ||
            !vel_params.count(kf_j.get())) {
            continue;
        }
        kf_j->imu_preintegrated->SetNewBias(kf_i->imu_bias);
        problem.AddResidualBlock(
            ImuPreintegrationGSCostFunctor::Create(
                *kf_j->imu_preintegrated, kf_i->GetImuPose(),
                kf_j->GetImuPose()),
            nullptr, vel_params[kf_i.get()].data(),
            vel_params[kf_j.get()].data(), bias.data(), rwg_aa.data(),
            &scale_param);
        ++edges;
    }
    if (edges == 0) {
        return;
    }

    if (prior_g > 0.f || prior_a > 0.f) {
        problem.AddResidualBlock(
            ImuBiasPriorCostFunctor::Create(
                Vec3::Zero(), Vec3::Zero(),
                prior_g > 0.f ? std::sqrt(static_cast<double>(prior_g)) : 0.0,
                prior_a > 0.f ? std::sqrt(static_cast<double>(prior_a)) : 0.0),
            nullptr, bias.data());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (covariance != nullptr) {
        covariance->resize(0, 0);
        std::vector<const double*> cov_blocks = {bias.data()};
        if (!fix_gravity_scale) {
            cov_blocks.push_back(rwg_aa.data());
            if (monocular) {
                cov_blocks.push_back(&scale_param);
            }
        }
        ceres::Covariance::Options cov_options;
        cov_options.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
        ceres::Covariance cov(cov_options);
        if (cov.Compute(cov_blocks, &problem)) {
            const int n = static_cast<int>(6 + (fix_gravity_scale ? 0 : 3) +
                                           ((!fix_gravity_scale && monocular) ? 1 : 0));
            covariance->setZero(n, n);
            double block[36];
            cov.GetCovarianceBlock(bias.data(), bias.data(), block);
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    (*covariance)(r, c) = block[r * 6 + c];
                }
            }
            int offset = 6;
            if (!fix_gravity_scale) {
                double rwg_block[9];
                cov.GetCovarianceBlock(rwg_aa.data(), rwg_aa.data(), rwg_block);
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        (*covariance)(offset + r, offset + c) = rwg_block[r * 3 + c];
                    }
                }
                offset += 3;
                if (monocular) {
                    double scale_var = 0.0;
                    cov.GetCovarianceBlock(&scale_param, &scale_param, &scale_var);
                    (*covariance)(offset, offset) = scale_var;
                }
            }
        }
    }

    *bg = Vec3(bias[0], bias[1], bias[2]);
    *ba = Vec3(bias[3], bias[4], bias[5]);
    *scale = scale_param;
    {
        const Eigen::Vector3d aa_vec(rwg_aa[0], rwg_aa[1], rwg_aa[2]);
        const double ang = aa_vec.norm();
        if (ang > 1e-12) {
            *Rwg = Eigen::AngleAxisd(ang, aa_vec / ang).toRotationMatrix();
        } else {
            *Rwg = Mat33::Identity();
        }
    }

    sensor::imu::Bias shared_bias;
    shared_bias.gyroscope = *bg;
    shared_bias.accelerometer = *ba;
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad() || !vel_params.count(kf.get())) {
            continue;
        }
        const auto& v = vel_params[kf.get()];
        kf->velocity_world = Vec3(v[0], v[1], v[2]);
        kf->has_velocity = true;
        const bool reintegrate =
            (kf->imu_bias.gyroscope - *bg).norm() > 0.01;
        kf->imu_bias = shared_bias;
        if (reintegrate && kf->imu_preintegrated) {
            kf->imu_preintegrated->SetNewBias(shared_bias);
            kf->imu_preintegrated->Reintegrate();
        } else if (kf->imu_preintegrated) {
            kf->imu_preintegrated->SetNewBias(shared_bias);
        }
    }
    // ApplyScaledRotation / SetImuInitialized left to LocalMapping (ORB flow).
}


void Optimizer::InertialOptimization(Map* map, Vec3* bg, Vec3* ba, float prior_g,
                                     float prior_a) {
    if (!map || !bg || !ba) {
        return;
    }
    Mat33 Rwg = Mat33::Identity();
    double scale = 1.0;
    InertialOptimization(map, &Rwg, &scale, bg, ba, /*monocular=*/false, prior_g,
                         prior_a, /*fix_gravity_scale=*/true, nullptr);
}

void Optimizer::WeldingBundleAdjustment(
    const std::shared_ptr<KeyFrame>& current_keyframe,
    const std::shared_ptr<KeyFrame>& merge_keyframe, Map* map) {
    if (!current_keyframe || !merge_keyframe || !map) {
        return;
    }
    LocalBundleAdjustment(current_keyframe, map);
    LocalBundleAdjustment(merge_keyframe, map);
}

void Optimizer::ScaleRefinement(Map* map, Mat33* Rwg, double* scale) {
    if (!map || !Rwg || !scale) {
        return;
    }
    auto keyframes = map->GetAllKeyFrames();
    if (keyframes.size() < 2) {
        return;
    }
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    // Poses stay in the cost (constants). Velocities and the shared bias are
    // parameter blocks only so the preintegration residual can read them.
    std::unordered_map<KeyFrame*, std::array<double, 3>> vel_params;
    ceres::Problem problem;
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        vel_params[kf.get()] = {kf->velocity_world.x(), kf->velocity_world.y(),
                                kf->velocity_world.z()};
        problem.AddParameterBlock(vel_params[kf.get()].data(), 3);
        problem.SetParameterBlockConstant(vel_params[kf.get()].data());
    }
    sensor::imu::Bias bias0;
    for (const auto& kf : keyframes) {
        if (kf && !kf->isBad()) {
            bias0 = kf->imu_bias;
            break;
        }
    }
    std::array<double, 6> bias = {
        bias0.gyroscope.x(),     bias0.gyroscope.y(),     bias0.gyroscope.z(),
        bias0.accelerometer.x(), bias0.accelerometer.y(), bias0.accelerometer.z()};
    problem.AddParameterBlock(bias.data(), 6);
    problem.SetParameterBlockConstant(bias.data());

    const Eigen::AngleAxisd aa_rwg(*Rwg);
    std::array<double, 3> rwg_aa = {
        aa_rwg.axis().x() * aa_rwg.angle(),
        aa_rwg.axis().y() * aa_rwg.angle(),
        aa_rwg.axis().z() * aa_rwg.angle()};
    double scale_param = (*scale > 1e-6) ? *scale : 1.0;
    problem.AddParameterBlock(rwg_aa.data(), 3);
    problem.AddParameterBlock(&scale_param, 1);

    int edges = 0;
    for (const auto& kf_j : keyframes) {
        if (!kf_j || kf_j->isBad() || !kf_j->imu_preintegrated) {
            continue;
        }
        auto kf_i = kf_j->previous_keyframe.lock();
        if (!kf_i || kf_i->isBad() || !vel_params.count(kf_i.get()) ||
            !vel_params.count(kf_j.get())) {
            continue;
        }
        problem.AddResidualBlock(
            ImuPreintegrationGSCostFunctor::Create(
                *kf_j->imu_preintegrated, kf_i->GetImuPose(),
                kf_j->GetImuPose()),
            new ceres::HuberLoss(1.0), vel_params[kf_i.get()].data(),
            vel_params[kf_j.get()].data(), bias.data(), rwg_aa.data(),
            &scale_param);
        ++edges;
    }
    if (edges == 0) {
        return;
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    *scale = scale_param;
    const Eigen::Vector3d aa_vec(rwg_aa[0], rwg_aa[1], rwg_aa[2]);
    const double ang = aa_vec.norm();
    if (ang > 1e-12) {
        *Rwg = Eigen::AngleAxisd(ang, aa_vec / ang).toRotationMatrix();
    } else {
        *Rwg = Mat33::Identity();
    }
}

namespace {

void AddUnaryPoseHessian(ceres::CostFunction* cost, const double* pose,
                         Eigen::Matrix<double, 15, 15>* information) {
    if (cost == nullptr || pose == nullptr || information == nullptr) {
        delete cost;
        return;
    }
    const int residual_size = cost->num_residuals();
    std::vector<double> residual(static_cast<size_t>(residual_size), 0.0);
    Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor> jacobian(
        residual_size, 6);
    double* jacobians[1] = {jacobian.data()};
    const double* parameters[1] = {pose};
    if (cost->Evaluate(parameters, residual.data(), jacobians)) {
        information->block<6, 6>(0, 0) += jacobian.transpose() * jacobian;
    }
    delete cost;
}

Eigen::Matrix<double, 15, 15> PoseImuSqrtInformation(
    const Eigen::Matrix<double, 15, 15>& information) {
    const Eigen::Matrix<double, 15, 15> symmetric =
        0.5 * (information + information.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> solver(
        symmetric);
    if (solver.info() != Eigen::Success) {
        return PoseImuPriorCostFunctor::DiagonalSqrtInformation();
    }
    Eigen::Matrix<double, 15, 1> eigenvalues = solver.eigenvalues();
    bool any_positive = false;
    for (int i = 0; i < 15; ++i) {
        if (eigenvalues[i] < 1e-12) {
            eigenvalues[i] = 0.0;
        } else {
            eigenvalues[i] = std::sqrt(eigenvalues[i]);
            any_positive = true;
        }
    }
    if (!any_positive) {
        return PoseImuPriorCostFunctor::DiagonalSqrtInformation();
    }
    return solver.eigenvectors() * eigenvalues.asDiagonal() *
           solver.eigenvectors().transpose();
}

int PoseInertialOptimizeImpl(
    tracking::Frame* frame, const sensor::imu::Preintegrator& pre,
    const SE3& Twb_prev, const Vec3& vel_prev_in,
    const sensor::imu::Bias& bias_prev_in, bool fix_prev_bias_to_frame_prior,
    bool reinit, const tracking::Frame::PoseImuPrior* prev_prior,
    tracking::Frame* previous_frame) {
    if (!frame || !frame->has_pose() || pre.delta_t < 1e-4) {
        return Optimizer::PoseOptimization(frame);
    }

    const sensor::imu::Calib calib =
        sensor::imu::DefaultCalibOr(frame->imu_calib);
    const Mat33 R_cb = calib.T_camera_body.rotation();
    const Vec3 t_cb = calib.T_camera_body.translation();

    double pose_prev[6];
    double pose_fr[6];
    Se3ToAngleAxisTranslation(Twb_prev, pose_prev);
    Se3ToAngleAxisTranslation(frame->GetImuPose(), pose_fr);

    double vel_prev[3] = {vel_prev_in.x(), vel_prev_in.y(), vel_prev_in.z()};
    double vel_fr[3] = {frame->velocity_world.x(), frame->velocity_world.y(),
                        frame->velocity_world.z()};
    if (!frame->has_velocity) {
        vel_fr[0] = vel_prev[0];
        vel_fr[1] = vel_prev[1];
        vel_fr[2] = vel_prev[2];
    }

    double bias_prev[6] = {bias_prev_in.gyroscope.x(),
                           bias_prev_in.gyroscope.y(),
                           bias_prev_in.gyroscope.z(),
                           bias_prev_in.accelerometer.x(),
                           bias_prev_in.accelerometer.y(),
                           bias_prev_in.accelerometer.z()};
    double bias_fr[6] = {frame->imu_bias.gyroscope.x(),
                         frame->imu_bias.gyroscope.y(),
                         frame->imu_bias.gyroscope.z(),
                         frame->imu_bias.accelerometer.x(),
                         frame->imu_bias.accelerometer.y(),
                         frame->imu_bias.accelerometer.z()};

    constexpr double kChi2Mono = 5.991;
    constexpr double kChi2Stereo = 7.815;
    const Vec3 gravity(0.0, 0.0, -sensor::imu::kGravity);

    int inliers = 0;
    for (int round = 0; round < 4; ++round) {
        ceres::Problem problem;
        problem.AddParameterBlock(pose_prev, 6);
        problem.AddParameterBlock(pose_fr, 6);
        problem.AddParameterBlock(vel_prev, 3);
        problem.AddParameterBlock(vel_fr, 3);
        problem.AddParameterBlock(bias_prev, 6);
        problem.AddParameterBlock(bias_fr, 6);
        const bool soft_prior = prev_prior != nullptr && prev_prior->valid;
        if (!soft_prior) {
            problem.SetParameterBlockConstant(pose_prev);
            problem.SetParameterBlockConstant(vel_prev);
            problem.SetParameterBlockConstant(bias_prev);
        } else {
            problem.AddResidualBlock(
                PoseImuPriorCostFunctor::Create(
                    prev_prior->Twb, prev_prior->velocity, prev_prior->bias,
                    prev_prior->sqrt_information),
                new ceres::HuberLoss(5.0), pose_prev, vel_prev, bias_prev);
        }

        problem.AddResidualBlock(
            ImuPreintegrationBiasCostFunctor::Create(pre, gravity), nullptr,
            pose_prev, pose_fr, vel_prev, vel_fr, bias_prev);
        problem.AddResidualBlock(
            ImuBiasWalkCostFunctor::CreateFromPreintegrator(pre), nullptr,
            bias_prev, bias_fr);
        if (fix_prev_bias_to_frame_prior) {
            problem.AddResidualBlock(
                ImuBiasPriorCostFunctor::Create(
                    bias_prev_in.gyroscope, bias_prev_in.accelerometer, 1e2,
                    1e3),
                nullptr, bias_fr);
        }

        const bool dual = frame->HasDualCameraIndex();
        const SE3 Trl = dual ? frame->T_c1_c2.inverse() : SE3Identity();
        const int n_feat = dual ? frame->TotalFeatures() : frame->num_keypoints;
        if (static_cast<int>(frame->outliers.size()) < n_feat) {
            frame->outliers.resize(static_cast<size_t>(n_feat), false);
        }

        for (int i = 0; i < n_feat; ++i) {
            if (i >= static_cast<int>(frame->map_points.size())) {
                break;
            }
            const auto& map_point = frame->map_points[static_cast<size_t>(i)];
            if (!map_point || map_point->isBad() ||
                (round > 0 && frame->outliers[static_cast<size_t>(i)])) {
                continue;
            }
            const Vec3 pw = map_point->GetWorldPos();
            const cv::KeyPoint kp = frame->GetKeyPoint(i);
            const double sqrt_info =
                std::sqrt(InvLevelSigma2(*frame, kp.octave));
            ceres::LossFunction* loss = nullptr;
            if (round < 2) {
                const bool stereo_obs =
                    !dual &&
                    i < static_cast<int>(frame->right_coordinate.size()) &&
                    frame->right_coordinate[static_cast<size_t>(i)] >= 0.f;
                loss = new ceres::HuberLoss(
                    std::sqrt(stereo_obs ? kChi2Stereo : kChi2Mono));
            }
            if (dual && i >= frame->num_left && frame->camera2) {
                problem.AddResidualBlock(
                    MonoPoseOnlyToBodyFromTwbCostFunctor::Create(
                        kp.pt.x, kp.pt.y, pw.x(), pw.y(), pw.z(),
                        ReprojCam::FromCamera(frame->camera2.get(), frame->fx,
                                              frame->fy, frame->cx, frame->cy),
                        sqrt_info, R_cb, t_cb, Trl.linear(), Trl.translation()),
                    loss, pose_fr);
            } else if (!dual &&
                       i < static_cast<int>(frame->right_coordinate.size()) &&
                       frame->right_coordinate[static_cast<size_t>(i)] >= 0.f) {
                problem.AddResidualBlock(
                    StereoPoseOnlyFromTwbCostFunctor::Create(
                        kp.pt.x, kp.pt.y,
                        frame->right_coordinate[static_cast<size_t>(i)], pw.x(),
                        pw.y(), pw.z(), frame->fx, frame->fy, frame->cx,
                        frame->cy, frame->baseline_times_fx, sqrt_info, R_cb,
                        t_cb),
                    loss, pose_fr);
            } else {
                problem.AddResidualBlock(
                    MonoPoseOnlyFromTwbCostFunctor::Create(
                        kp.pt.x, kp.pt.y, pw.x(), pw.y(), pw.z(),
                        ReprojCam::FromCamera(frame->camera.get(), frame->fx,
                                              frame->fy, frame->cx, frame->cy),
                        sqrt_info, R_cb, t_cb),
                    loss, pose_fr);
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        inliers = 0;
        const double chi2_mono_thr[4] = {12.0, 7.5, 5.991, 5.991};
        const double chi2_stereo_thr[4] = {15.6, 9.8, 7.815, 7.815};
        const SE3 Tcw = sensor::imu::ImuPoseToCameraPose(
            AngleAxisTranslationToSe3(pose_fr), calib);
        for (int i = 0; i < n_feat; ++i) {
            if (i >= static_cast<int>(frame->map_points.size())) {
                break;
            }
            const auto& map_point = frame->map_points[static_cast<size_t>(i)];
            if (!map_point || map_point->isBad()) {
                continue;
            }
            Vec3 pc = Tcw * map_point->GetWorldPos();
            const bool right_body = dual && i >= frame->num_left;
            if (right_body) {
                pc = Trl * pc;
            }
            const cv::KeyPoint kp = frame->GetKeyPoint(i);
            const double sqrt_info =
                std::sqrt(InvLevelSigma2(*frame, kp.octave));
            const sensor::GeometricCamera* geo =
                right_body ? frame->camera2.get() : frame->camera.get();
            double pu = 0.0;
            double pv = 0.0;
            if (geo) {
                const Vec2 uv = geo->Project(pc);
                pu = uv.x();
                pv = uv.y();
            } else {
                if (pc.z() <= 1e-6) {
                    frame->outliers[static_cast<size_t>(i)] = true;
                    continue;
                }
                const double inv_z = 1.0 / pc.z();
                pu = frame->fx * pc.x() * inv_z + frame->cx;
                pv = frame->fy * pc.y() * inv_z + frame->cy;
            }
            if (!dual &&
                i < static_cast<int>(frame->right_coordinate.size()) &&
                frame->right_coordinate[static_cast<size_t>(i)] >= 0.f) {
                if (pc.z() <= 1e-6) {
                    frame->outliers[static_cast<size_t>(i)] = true;
                    continue;
                }
                const double inv_z = 1.0 / pc.z();
                const double pur = pu - frame->baseline_times_fx * inv_z;
                const double eu = (pu - kp.pt.x) * sqrt_info;
                const double ev = (pv - kp.pt.y) * sqrt_info;
                const double er =
                    (pur - frame->right_coordinate[static_cast<size_t>(i)]) *
                    sqrt_info;
                frame->outliers[static_cast<size_t>(i)] =
                    (eu * eu + ev * ev + er * er) > chi2_stereo_thr[round];
            } else {
                const double eu = (pu - kp.pt.x) * sqrt_info;
                const double ev = (pv - kp.pt.y) * sqrt_info;
                const bool close = map_point->track_depth < 10.f;
                const double thr = close ? 1.5 * chi2_mono_thr[round]
                                         : chi2_mono_thr[round];
                frame->outliers[static_cast<size_t>(i)] =
                    (eu * eu + ev * ev) > thr || pc.z() <= 1e-6;
            }
            if (!frame->outliers[static_cast<size_t>(i)]) {
                ++inliers;
            }
        }
    }

    if (inliers < 30 && !reinit) {
        constexpr double kChi2MonoOut = 18.0;
        constexpr double kChi2StereoOut = 24.0;
        const SE3 Tcw = sensor::imu::ImuPoseToCameraPose(
            AngleAxisTranslationToSe3(pose_fr), calib);
        const bool dual = frame->HasDualCameraIndex();
        const SE3 Trl = dual ? frame->T_c1_c2.inverse() : SE3Identity();
        const int n_feat = dual ? frame->TotalFeatures() : frame->num_keypoints;
        inliers = 0;
        for (int i = 0; i < n_feat; ++i) {
            if (i >= static_cast<int>(frame->map_points.size())) {
                break;
            }
            const auto& map_point = frame->map_points[static_cast<size_t>(i)];
            if (!map_point || map_point->isBad()) {
                continue;
            }
            Vec3 pc = Tcw * map_point->GetWorldPos();
            const bool right_body = dual && i >= frame->num_left;
            if (right_body) {
                pc = Trl * pc;
            }
            if (pc.z() <= 1e-6) {
                frame->outliers[static_cast<size_t>(i)] = true;
                continue;
            }
            const cv::KeyPoint kp = frame->GetKeyPoint(i);
            const double sqrt_info =
                std::sqrt(InvLevelSigma2(*frame, kp.octave));
            const sensor::GeometricCamera* geo =
                right_body ? frame->camera2.get() : frame->camera.get();
            double pu = 0.0;
            double pv = 0.0;
            if (geo) {
                const Vec2 uv = geo->Project(pc);
                pu = uv.x();
                pv = uv.y();
            } else {
                const double inv_z = 1.0 / pc.z();
                pu = frame->fx * pc.x() * inv_z + frame->cx;
                pv = frame->fy * pc.y() * inv_z + frame->cy;
            }
            const double eu = (pu - kp.pt.x) * sqrt_info;
            const double ev = (pv - kp.pt.y) * sqrt_info;
            const bool stereo =
                !dual && i < static_cast<int>(frame->right_coordinate.size()) &&
                frame->right_coordinate[static_cast<size_t>(i)] >= 0.f;
            if (stereo) {
                const double pur =
                    pu - frame->baseline_times_fx * (1.0 / pc.z());
                const double er =
                    (pur - frame->right_coordinate[static_cast<size_t>(i)]) *
                    sqrt_info;
                frame->outliers[static_cast<size_t>(i)] =
                    (eu * eu + ev * ev + er * er) >= kChi2StereoOut;
            } else {
                frame->outliers[static_cast<size_t>(i)] =
                    (eu * eu + ev * ev) >= kChi2MonoOut;
            }
            if (!frame->outliers[static_cast<size_t>(i)]) {
                ++inliers;
            }
        }
    }

    frame->SetImuPose(AngleAxisTranslationToSe3(pose_fr));
    frame->pose_imu_prior.Twb = frame->GetImuPose();
    frame->pose_imu_prior.velocity = Vec3(vel_fr[0], vel_fr[1], vel_fr[2]);
    frame->pose_imu_prior.bias.gyroscope = Vec3(bias_fr[0], bias_fr[1], bias_fr[2]);
    frame->pose_imu_prior.bias.accelerometer =
        Vec3(bias_fr[3], bias_fr[4], bias_fr[5]);
    {
        Eigen::Matrix<double, 15, 15> information =
            Eigen::Matrix<double, 15, 15>::Zero();
        ceres::CostFunction* imu_cost =
            ImuPreintegrationBiasCostFunctor::Create(pre, gravity);
        double imu_residual[9];
        Eigen::Matrix<double, 9, 6, Eigen::RowMajor> jacobian_pose;
        Eigen::Matrix<double, 9, 3, Eigen::RowMajor> jacobian_velocity;
        double* imu_jacobians[5] = {nullptr, jacobian_pose.data(), nullptr,
                                    jacobian_velocity.data(), nullptr};
        const double* imu_parameters[5] = {pose_prev, pose_fr, vel_prev, vel_fr,
                                           bias_prev};
        if (imu_cost->Evaluate(imu_parameters, imu_residual, imu_jacobians)) {
            Eigen::Matrix<double, 9, 9> jacobian;
            jacobian.leftCols(6) = jacobian_pose;
            jacobian.rightCols(3) = jacobian_velocity;
            information.block<9, 9>(0, 0) += jacobian.transpose() * jacobian;
        }
        delete imu_cost;

        ceres::CostFunction* bias_walk =
            ImuBiasWalkCostFunctor::CreateFromPreintegrator(pre);
        double bias_residual[6];
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian_bias;
        double* bias_jacobians[2] = {nullptr, jacobian_bias.data()};
        const double* bias_parameters[2] = {bias_prev, bias_fr};
        if (bias_walk->Evaluate(bias_parameters, bias_residual, bias_jacobians)) {
            information.block<6, 6>(9, 9) +=
                jacobian_bias.transpose() * jacobian_bias;
        }
        delete bias_walk;

        const bool dual_view = frame->HasDualCameraIndex();
        const SE3 right_from_left =
            dual_view ? frame->T_c1_c2.inverse() : SE3Identity();
        const int feature_count =
            dual_view ? frame->TotalFeatures() : frame->num_keypoints;
        for (int i = 0; i < feature_count; ++i) {
            if (i >= static_cast<int>(frame->map_points.size()) ||
                i >= static_cast<int>(frame->outliers.size()) ||
                frame->outliers[static_cast<size_t>(i)]) {
                continue;
            }
            const auto& map_point = frame->map_points[static_cast<size_t>(i)];
            if (!map_point || map_point->isBad()) {
                continue;
            }
            const Vec3 point = map_point->GetWorldPos();
            const cv::KeyPoint keypoint = frame->GetKeyPoint(i);
            const double weight =
                std::sqrt(InvLevelSigma2(*frame, keypoint.octave));
            ceres::CostFunction* visual = nullptr;
            if (dual_view && i >= frame->num_left && frame->camera2) {
                visual = MonoPoseOnlyToBodyFromTwbCostFunctor::Create(
                    keypoint.pt.x, keypoint.pt.y, point.x(), point.y(),
                    point.z(),
                    ReprojCam::FromCamera(frame->camera2.get(), frame->fx,
                                          frame->fy, frame->cx, frame->cy),
                    weight, R_cb, t_cb, right_from_left.linear(),
                    right_from_left.translation());
            } else if (!dual_view &&
                       i < static_cast<int>(frame->right_coordinate.size()) &&
                       frame->right_coordinate[static_cast<size_t>(i)] >= 0.f) {
                visual = StereoPoseOnlyFromTwbCostFunctor::Create(
                    keypoint.pt.x, keypoint.pt.y,
                    frame->right_coordinate[static_cast<size_t>(i)], point.x(),
                    point.y(), point.z(), frame->fx, frame->fy, frame->cx,
                    frame->cy, frame->baseline_times_fx, weight, R_cb, t_cb);
            } else {
                visual = MonoPoseOnlyFromTwbCostFunctor::Create(
                    keypoint.pt.x, keypoint.pt.y, point.x(), point.y(),
                    point.z(),
                    ReprojCam::FromCamera(frame->camera.get(), frame->fx,
                                          frame->fy, frame->cx, frame->cy),
                    weight, R_cb, t_cb);
            }
            AddUnaryPoseHessian(visual, pose_fr, &information);
        }
        frame->pose_imu_prior.sqrt_information =
            PoseImuSqrtInformation(information);
    }
    frame->pose_imu_prior.valid = true;
    frame->velocity_world = Vec3(vel_fr[0], vel_fr[1], vel_fr[2]);
    frame->has_velocity = true;
    frame->imu_bias.gyroscope = Vec3(bias_fr[0], bias_fr[1], bias_fr[2]);
    frame->imu_bias.accelerometer = Vec3(bias_fr[3], bias_fr[4], bias_fr[5]);
    if (previous_frame != nullptr && prev_prior != nullptr && prev_prior->valid) {
        previous_frame->SetImuPose(AngleAxisTranslationToSe3(pose_prev));
        previous_frame->velocity_world = Vec3(vel_prev[0], vel_prev[1], vel_prev[2]);
        previous_frame->has_velocity = true;
        previous_frame->imu_bias.gyroscope =
            Vec3(bias_prev[0], bias_prev[1], bias_prev[2]);
        previous_frame->imu_bias.accelerometer =
            Vec3(bias_prev[3], bias_prev[4], bias_prev[5]);
    }
    return inliers;
}

}  // namespace

int Optimizer::PoseInertialOptimizationLastKeyFrame(
    tracking::Frame* frame, const std::shared_ptr<KeyFrame>& last_keyframe,
    bool reinit) {
    if (!frame || !last_keyframe || !frame->imu_preintegrated) {
        return PoseOptimization(frame);
    }
    const int inliers = PoseInertialOptimizeImpl(
        frame, *frame->imu_preintegrated, last_keyframe->GetImuPose(),
        last_keyframe->has_velocity ? last_keyframe->velocity_world
                                    : Vec3::Zero(),
        last_keyframe->imu_bias, /*fix_prev_bias_to_frame_prior=*/true, reinit,
        nullptr, nullptr);
    if (frame->imu_preintegrated) {
        frame->imu_preintegrated->SetNewBias(frame->imu_bias);
    }
    return inliers;
}

int Optimizer::PoseInertialOptimizationLastFrame(tracking::Frame* frame,
                                                 tracking::Frame* last_frame,
                                                 bool reinit) {
    if (!frame || !last_frame || !frame->has_pose() || !last_frame->has_pose()) {
        return PoseOptimization(frame);
    }
    if (!frame->imu_preintegrated_from_last_frame) {
        if (frame->imu_preintegrated && frame->reference_keyframe) {
            return PoseInertialOptimizationLastKeyFrame(
                frame, frame->reference_keyframe, reinit);
        }
        return PoseOptimization(frame);
    }
    const int inliers = PoseInertialOptimizeImpl(
        frame, *frame->imu_preintegrated_from_last_frame,
        last_frame->GetImuPose(),
        last_frame->has_velocity ? last_frame->velocity_world : Vec3::Zero(),
        last_frame->imu_bias, /*fix_prev_bias_to_frame_prior=*/false, reinit,
        &last_frame->pose_imu_prior, last_frame);
    if (!last_frame->pose_imu_prior.valid) {
        last_frame->imu_bias = frame->imu_bias;
    }
    if (frame->imu_preintegrated_from_last_frame) {
        frame->imu_preintegrated_from_last_frame->SetNewBias(frame->imu_bias);
    }
    if (frame->imu_preintegrated) {
        frame->imu_preintegrated->SetNewBias(frame->imu_bias);
    }
    return inliers;
}

void Optimizer::FullInertialBA(Map* map, int iterations, bool fix_local,
                               bool* stop_flag, bool init_shared_bias,
                               float prior_g, float prior_a, uint64_t gba_id) {
    if (!map) {
        return;
    }
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }
    auto keyframes = map->GetAllKeyFrames();
    if (keyframes.size() < 3) {
        return;
    }
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    std::unordered_map<KeyFrame*, std::array<double, 3>> vel_params;
    std::unordered_map<KeyFrame*, std::array<double, 6>> bias_params;
    ceres::Problem problem;
    const Vec3 gravity(0.0, 0.0, -sensor::imu::kGravity);

    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        Se3ToAngleAxisTranslation(kf->GetImuPose(),
                                  pose_params[kf.get()].data());
        problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
        vel_params[kf.get()] = {kf->velocity_world.x(), kf->velocity_world.y(),
                                kf->velocity_world.z()};
        problem.AddParameterBlock(vel_params[kf.get()].data(), 3);
        if (!init_shared_bias) {
            bias_params[kf.get()] = {
                kf->imu_bias.gyroscope.x(),     kf->imu_bias.gyroscope.y(),
                kf->imu_bias.gyroscope.z(),     kf->imu_bias.accelerometer.x(),
                kf->imu_bias.accelerometer.y(), kf->imu_bias.accelerometer.z()};
            problem.AddParameterBlock(bias_params[kf.get()].data(), 6);
        }
        if (fix_local && kf == keyframes.front()) {
            problem.SetParameterBlockConstant(pose_params[kf.get()].data());
            problem.SetParameterBlockConstant(vel_params[kf.get()].data());
        }
    }

    double shared_bias[6] = {0, 0, 0, 0, 0, 0};
    if (init_shared_bias && !keyframes.empty()) {
        const auto& kf0 = keyframes.back();
        shared_bias[0] = kf0->imu_bias.gyroscope.x();
        shared_bias[1] = kf0->imu_bias.gyroscope.y();
        shared_bias[2] = kf0->imu_bias.gyroscope.z();
        shared_bias[3] = kf0->imu_bias.accelerometer.x();
        shared_bias[4] = kf0->imu_bias.accelerometer.y();
        shared_bias[5] = kf0->imu_bias.accelerometer.z();
        problem.AddParameterBlock(shared_bias, 6);
        problem.AddResidualBlock(
            ImuBiasPriorCostFunctor::Create(Vec3::Zero(), Vec3::Zero(), prior_g,
                                            prior_a),
            nullptr, shared_bias);
    }

    for (size_t i = 1; i < keyframes.size(); ++i) {
        const auto& kf_j = keyframes[i];
        auto kf_i = kf_j->previous_keyframe.lock();
        if (!kf_i || !kf_j->imu_preintegrated ||
            pose_params.count(kf_i.get()) == 0 ||
            pose_params.count(kf_j.get()) == 0) {
            continue;
        }
        double* bias_i =
            init_shared_bias ? shared_bias : bias_params[kf_i.get()].data();
        problem.AddResidualBlock(
            ImuPreintegrationBiasCostFunctor::Create(*kf_j->imu_preintegrated,
                                                     gravity),
            nullptr, pose_params[kf_i.get()].data(),
            pose_params[kf_j.get()].data(), vel_params[kf_i.get()].data(),
            vel_params[kf_j.get()].data(), bias_i);
        if (!init_shared_bias && bias_params.count(kf_j.get())) {
            problem.AddResidualBlock(
                ImuBiasWalkCostFunctor::CreateFromPreintegrator(
                    *kf_j->imu_preintegrated),
                nullptr, bias_params[kf_i.get()].data(),
                bias_params[kf_j.get()].data());
        }
    }

    std::unordered_map<MapPoint*, std::array<double, 3>> point_params;
    AddInertialVisualEdges(&problem, keyframes, &pose_params, &point_params,
                           nullptr);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = iterations;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (stop_flag != nullptr && *stop_flag) {
        return;
    }

    for (const auto& kf : keyframes) {
        if (!kf || pose_params.count(kf.get()) == 0) {
            continue;
        }
        const SE3 Twb =
            AngleAxisTranslationToSe3(pose_params[kf.get()].data());
        const auto& v = vel_params[kf.get()];
        sensor::imu::Bias bias = kf->imu_bias;
        if (init_shared_bias) {
            bias.gyroscope = Vec3(shared_bias[0], shared_bias[1], shared_bias[2]);
            bias.accelerometer =
                Vec3(shared_bias[3], shared_bias[4], shared_bias[5]);
        } else if (bias_params.count(kf.get())) {
            const auto& b = bias_params[kf.get()];
            bias.gyroscope = Vec3(b[0], b[1], b[2]);
            bias.accelerometer = Vec3(b[3], b[4], b[5]);
        }
        if (gba_id == 0) {
            kf->SetImuPose(Twb);
            kf->velocity_world = Vec3(v[0], v[1], v[2]);
            kf->has_velocity = true;
            kf->imu_bias = bias;
            if (kf->imu_preintegrated) {
                kf->imu_preintegrated->SetNewBias(kf->imu_bias);
            }
            kf->imu_ready = true;
        } else {
            kf->pose_gba = sensor::imu::ImuPoseToCameraPose(
                Twb, sensor::imu::DefaultCalibOr(kf->imu_calib));
            kf->velocity_gba = Vec3(v[0], v[1], v[2]);
            kf->bias_gba = bias;
            kf->gba_for_kf = gba_id;
        }
    }
    for (const auto& [mp, p] : point_params) {
        if (!mp || mp->isBad()) {
            continue;
        }
        const Vec3 pw(p[0], p[1], p[2]);
        if (gba_id == 0) {
            mp->SetWorldPos(pw);
            mp->UpdateNormalAndDepth();
        } else {
            mp->position_gba = pw;
            mp->gba_for_kf = gba_id;
        }
    }
}

void Optimizer::MergeInertialBA(
    const std::shared_ptr<KeyFrame>& current_keyframe,
    const std::shared_ptr<KeyFrame>& merge_keyframe, Map* map) {
    if (!current_keyframe || !merge_keyframe || !map) {
        return;
    }
    if (!current_keyframe->imu_preintegrated &&
        !merge_keyframe->imu_preintegrated) {
        WeldingBundleAdjustment(current_keyframe, merge_keyframe, map);
        return;
    }

    constexpr int kHalf = 6;
    auto walk_prev = [](const std::shared_ptr<KeyFrame>& start, int count,
                        std::vector<std::shared_ptr<KeyFrame>>* out) {
        auto cursor = start;
        for (int i = 0; i < count && cursor && !cursor->isBad(); ++i) {
            out->push_back(cursor);
            cursor = cursor->previous_keyframe.lock();
        }
        return cursor;
    };

    std::vector<std::shared_ptr<KeyFrame>> temporal;
    walk_prev(current_keyframe, kHalf, &temporal);
    std::vector<std::shared_ptr<KeyFrame>> merge_chain;
    auto merge_before = walk_prev(merge_keyframe, kHalf / 2, &merge_chain);
    std::shared_ptr<KeyFrame> fixed_anchor;
    if (merge_before && !merge_before->isBad()) {
        fixed_anchor = merge_before;
    } else if (merge_chain.size() > 1) {
        fixed_anchor = merge_chain.back();
        merge_chain.pop_back();
    }
    auto next = merge_keyframe->next_keyframe.lock();
    while (next && !next->isBad() &&
           temporal.size() + merge_chain.size() < static_cast<size_t>(2 * kHalf)) {
        merge_chain.push_back(next);
        next = next->next_keyframe.lock();
    }

    std::unordered_set<KeyFrame*> seen;
    std::vector<std::shared_ptr<KeyFrame>> optimizable;
    for (const auto& kf : temporal) {
        if (kf && seen.insert(kf.get()).second) {
            optimizable.push_back(kf);
        }
    }
    for (const auto& kf : merge_chain) {
        if (kf && seen.insert(kf.get()).second) {
            optimizable.push_back(kf);
        }
    }
    if (optimizable.size() < 2) {
        LocalInertialBA(current_keyframe, map);
        return;
    }

    auto add_imu_state = [](ceres::Problem* problem, KeyFrame* kf,
                            std::unordered_map<KeyFrame*, std::array<double, 6>>*
                                pose_params,
                            std::unordered_map<KeyFrame*, std::array<double, 3>>*
                                vel_params,
                            std::unordered_map<KeyFrame*, std::array<double, 6>>*
                                bias_params,
                            bool fixed) {
        Se3ToAngleAxisTranslation(kf->GetImuPose(), (*pose_params)[kf].data());
        problem->AddParameterBlock((*pose_params)[kf].data(), 6);
        (*vel_params)[kf] = {kf->velocity_world.x(), kf->velocity_world.y(),
                             kf->velocity_world.z()};
        problem->AddParameterBlock((*vel_params)[kf].data(), 3);
        (*bias_params)[kf] = {
            kf->imu_bias.gyroscope.x(),     kf->imu_bias.gyroscope.y(),
            kf->imu_bias.gyroscope.z(),     kf->imu_bias.accelerometer.x(),
            kf->imu_bias.accelerometer.y(), kf->imu_bias.accelerometer.z()};
        problem->AddParameterBlock((*bias_params)[kf].data(), 6);
        if (fixed) {
            problem->SetParameterBlockConstant((*pose_params)[kf].data());
            problem->SetParameterBlockConstant((*vel_params)[kf].data());
            problem->SetParameterBlockConstant((*bias_params)[kf].data());
        }
    };

    std::unordered_map<KeyFrame*, std::array<double, 6>> pose_params;
    std::unordered_map<KeyFrame*, std::array<double, 3>> vel_params;
    std::unordered_map<KeyFrame*, std::array<double, 6>> bias_params;
    ceres::Problem problem;
    for (const auto& kf : optimizable) {
        add_imu_state(&problem, kf.get(), &pose_params, &vel_params,
                      &bias_params, false);
    }
    if (fixed_anchor && seen.insert(fixed_anchor.get()).second) {
        add_imu_state(&problem, fixed_anchor.get(), &pose_params, &vel_params,
                      &bias_params, true);
    }

    std::vector<std::shared_ptr<MapPoint>> local_points;
    std::unordered_set<MapPoint*> seen_points;
    for (const auto& kf : optimizable) {
        for (const auto& mp : kf->GetMapPoints()) {
            if (!mp || mp->isBad() || !seen_points.insert(mp.get()).second) {
                continue;
            }
            local_points.push_back(mp);
        }
    }
    int cov_count = 0;
    for (const auto& mp : local_points) {
        if (cov_count >= 30) {
            break;
        }
        for (const auto& [weak_kf, index] : mp->GetObservations()) {
            (void)index;
            auto kf = weak_kf.lock();
            if (!kf || kf->isBad() || seen.count(kf.get())) {
                continue;
            }
            seen.insert(kf.get());
            Se3ToAngleAxisTranslation(kf->GetImuPose(),
                                      pose_params[kf.get()].data());
            problem.AddParameterBlock(pose_params[kf.get()].data(), 6);
            ++cov_count;
            break;
        }
    }

    const Vec3 gravity(0.0, 0.0, -sensor::imu::kGravity);
    for (const auto& kf_j : optimizable) {
        if (!kf_j || !kf_j->imu_preintegrated) {
            continue;
        }
        auto kf_i = kf_j->previous_keyframe.lock();
        if (!kf_i || bias_params.count(kf_i.get()) == 0 ||
            bias_params.count(kf_j.get()) == 0) {
            continue;
        }
        problem.AddResidualBlock(
            ImuPreintegrationBiasCostFunctor::Create(*kf_j->imu_preintegrated,
                                                     gravity),
            nullptr, pose_params[kf_i.get()].data(),
            pose_params[kf_j.get()].data(), vel_params[kf_i.get()].data(),
            vel_params[kf_j.get()].data(), bias_params[kf_i.get()].data());
        problem.AddResidualBlock(
            ImuBiasWalkCostFunctor::CreateFromPreintegrator(
                *kf_j->imu_preintegrated),
            nullptr, bias_params[kf_i.get()].data(),
            bias_params[kf_j.get()].data());
    }

    std::unordered_map<MapPoint*, std::array<double, 3>> point_params;
    AddInertialVisualEdges(&problem, optimizable, &pose_params, &point_params,
                           nullptr);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 10;
    options.initial_trust_region_radius = 1e3;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (const auto& kf : optimizable) {
        if (!kf || pose_params.count(kf.get()) == 0 ||
            bias_params.count(kf.get()) == 0) {
            continue;
        }
        kf->SetImuPose(AngleAxisTranslationToSe3(pose_params[kf.get()].data()));
        const auto& v = vel_params[kf.get()];
        kf->velocity_world = Vec3(v[0], v[1], v[2]);
        kf->has_velocity = true;
        const auto& b = bias_params[kf.get()];
        kf->imu_bias.gyroscope = Vec3(b[0], b[1], b[2]);
        kf->imu_bias.accelerometer = Vec3(b[3], b[4], b[5]);
        if (kf->imu_preintegrated) {
            kf->imu_preintegrated->SetNewBias(kf->imu_bias);
        }
    }
    for (const auto& [mp, p] : point_params) {
        if (!mp || mp->isBad()) {
            continue;
        }
        mp->SetWorldPos(Vec3(p[0], p[1], p[2]));
        mp->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraph4DoF(
    Map* map, const std::shared_ptr<KeyFrame>& loop_keyframe,
    const std::shared_ptr<KeyFrame>& current_keyframe,
    const SE3& loop_relative_pose_cw,
    const std::unordered_map<KeyFrame*, SE3>* pose_before) {
    if (!map || !loop_keyframe || !current_keyframe) {
        return;
    }
    // Gravity-aligned: optimize yaw + translation only (ORB-SLAM3 4DoF).
    const auto keyframes = map->GetAllKeyFrames();
    if (keyframes.size() < 2) {
        return;
    }

    std::unordered_map<KeyFrame*, Mat33> R_fixed;
    std::unordered_map<KeyFrame*, std::array<double, 4>> pose4;
    ceres::Problem problem;

    auto YawFromR = [](const Mat33& R) {
        return std::atan2(R(1, 0), R(0, 0));
    };
    auto RFromYawFixed = [](double yaw, const Mat33& R0) {
        const double y0 = std::atan2(R0(1, 0), R0(0, 0));
        const Mat33 dR =
            Eigen::AngleAxisd(yaw - y0, Vec3::UnitZ()).toRotationMatrix();
        return dR * R0;
    };

    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        const SE3 Tcw = kf->GetPose();
        R_fixed[kf.get()] = Tcw.rotation();
        pose4[kf.get()] = {YawFromR(Tcw.rotation()), Tcw.translation().x(),
                           Tcw.translation().y(), Tcw.translation().z()};
        problem.AddParameterBlock(pose4[kf.get()].data(), 4);
    }
    if (pose4.count(loop_keyframe.get())) {
        problem.SetParameterBlockConstant(pose4[loop_keyframe.get()].data());
    }

    auto AddEdge = [&](KeyFrame* a, KeyFrame* b, const SE3& T_ab,
                       double weight) {
        if (!a || !b || pose4.count(a) == 0 || pose4.count(b) == 0) {
            return;
        }
        problem.AddResidualBlock(
            RelativeYawTranslationCostFunctor::Create(
                T_ab.rotation(), T_ab.translation(), R_fixed[a], R_fixed[b],
                weight),
            nullptr, pose4[a].data(), pose4[b].data());
    };

    auto PoseBefore = [&](const std::shared_ptr<KeyFrame>& kf) {
        if (pose_before != nullptr) {
            const auto it = pose_before->find(kf.get());
            if (it != pose_before->end()) {
                return it->second;
            }
        }
        return kf->GetPose();
    };
    std::set<std::pair<KeyFrame*, KeyFrame*>> linked;
    auto AddMeasured = [&](const std::shared_ptr<KeyFrame>& a,
                           const std::shared_ptr<KeyFrame>& b, double weight) {
        if (!a || !b || a.get() == b.get()) {
            return;
        }
        KeyFrame* lo = a.get() < b.get() ? a.get() : b.get();
        KeyFrame* hi = a.get() < b.get() ? b.get() : a.get();
        if (weight <= 1.0 && !linked.insert({lo, hi}).second) {
            return;
        }
        AddEdge(a.get(), b.get(), PoseBefore(a) * PoseBefore(b).inverse(),
                weight);
    };
    for (const auto& keyframe : keyframes) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        for (const auto& neighbor :
             keyframe->GetBestCovisibilityKeyFrames(15)) {
            if (!neighbor || neighbor->id <= keyframe->id) {
                continue;
            }
            AddMeasured(keyframe, neighbor, 1.0);
        }
        AddMeasured(keyframe, keyframe->GetParent(), 1.0);
        for (const auto& loop_kf : keyframe->GetLoopEdges()) {
            AddMeasured(keyframe, loop_kf, 1.0);
        }
    }
    AddEdge(current_keyframe.get(), loop_keyframe.get(), loop_relative_pose_cw,
            10.0);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::unordered_map<KeyFrame*, SE3> old_poses;
    for (const auto& kf : keyframes) {
        if (!kf || pose4.count(kf.get()) == 0) {
            continue;
        }
        old_poses[kf.get()] = kf->GetPose();
        const auto& p = pose4[kf.get()];
        SE3 Tcw = SE3Identity();
        Tcw.linear() = RFromYawFixed(p[0], R_fixed[kf.get()]);
        Tcw.translation() = Vec3(p[1], p[2], p[3]);
        kf->SetPose(Tcw);
    }
    for (const auto& map_point : map->GetAllMapPoints()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        auto observations = map_point->GetObservations();
        if (observations.empty()) {
            continue;
        }
        auto ref = observations.begin()->first.lock();
        if (!ref || old_poses.count(ref.get()) == 0) {
            continue;
        }
        const Vec3 Pc = old_poses[ref.get()] * map_point->GetWorldPos();
        map_point->SetWorldPos(ref->GetPoseInverse() * Pc);
        map_point->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeLidarPose(SE3* T_wb,
                                 const std::vector<LidarPlaneFactor>& factors,
                                 const SE3* pose_prior, double prior_sqrt_info,
                                 int max_iterations) {
    if (T_wb == nullptr || factors.size() < 5) {
        return 0;
    }

    double pose6[6];
    Se3ToAngleAxisTranslation(*T_wb, pose6);
    std::vector<char> inlier(factors.size(), 1);

    for (int round = 0; round < 2; ++round) {
        ceres::Problem problem;
        problem.AddParameterBlock(pose6, 6);
        if (pose_prior != nullptr && prior_sqrt_info > 0.0) {
            problem.AddResidualBlock(
                PosePriorCostFunctor::Create(*pose_prior, prior_sqrt_info),
                nullptr, pose6);
        }
        int used = 0;
        for (std::size_t i = 0; i < factors.size(); ++i) {
            if (!inlier[i]) {
                continue;
            }
            problem.AddResidualBlock(
                LidarPointToPlaneCostFunctor::Create(factors[i]),
                new ceres::HuberLoss(0.1), pose6);
            ++used;
        }
        if (used < 5) {
            break;
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = max_iterations > 0 ? max_iterations : 10;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        const SE3 pose = AngleAxisTranslationToSe3(pose6);
        for (std::size_t i = 0; i < factors.size(); ++i) {
            const Vec3 pw = pose * factors[i].point_body;
            const double residual =
                factors[i].plane_normal.dot(pw - factors[i].plane_point);
            inlier[i] = std::fabs(residual) < 0.2 ? 1 : 0;
        }
    }

    *T_wb = AngleAxisTranslationToSe3(pose6);
    int inliers = 0;
    for (char flag : inlier) {
        inliers += flag ? 1 : 0;
    }
    return inliers;
}

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
