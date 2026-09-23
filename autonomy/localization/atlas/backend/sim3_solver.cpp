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
 * @file sim3_solver.cpp
 * @brief Closed-form Horn Sim3 + RANSAC (aligned with ORB-SLAM3 Sim3Solver.cc).
 */

#include "autonomy/localization/atlas/backend/sim3_solver.hpp"

#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

Sim3Solver::Sim3Solver(
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2,
    const std::vector<std::shared_ptr<MapPoint>>& matches12, bool fix_scale)
    : keyframe1_(keyframe1), keyframe2_(keyframe2), fix_scale_(fix_scale) {
    if (!keyframe1_ || !keyframe2_) {
        return;
    }
    const SE3 Tcw1 = keyframe1_->GetPose();
    const SE3 Tcw2 = keyframe2_->GetPose();
    const auto map_points1 = keyframe1_->GetMapPoints();

    for (size_t i = 0; i < matches12.size(); ++i) {
        const auto& match = matches12[i];
        if (!match || match->isBad()) {
            continue;
        }
        if (i >= map_points1.size() || !map_points1[i] ||
            map_points1[i]->isBad()) {
            continue;
        }
        const Vec3 Xc1 = Tcw1 * map_points1[i]->GetWorldPos();
        const Vec3 Xc2 = Tcw2 * match->GetWorldPos();
        if (Xc1.z() <= 0.0 || Xc2.z() <= 0.0) {
            continue;
        }
        points_camera1_.push_back(Xc1);
        points_camera2_.push_back(Xc2);
        indices1_.push_back(i);
    }
    num_matches_ = static_cast<int>(points_camera1_.size());
    current_inliers_.assign(num_matches_, false);
    best_inliers_.assign(matches12.size(), false);
}

void Sim3Solver::SetRansacParameters(double probability, int min_inliers,
                                     int max_iterations) {
    ransac_probability_ = probability;
    ransac_min_inliers_ = min_inliers;
    ransac_max_iterations_ = max_iterations;
}

void Sim3Solver::ComputeCentroid(const Eigen::Matrix3d& P, Eigen::Matrix3d* Pr,
                                 Vec3* centroid) const {
    *centroid = P.rowwise().mean();
    for (int i = 0; i < 3; ++i) {
        Pr->col(i) = P.col(i) - *centroid;
    }
}

void Sim3Solver::ComputeSim3(const Eigen::Matrix3d& P1,
                             const Eigen::Matrix3d& P2) {
    Eigen::Matrix3d Pr1;
    Eigen::Matrix3d Pr2;
    Vec3 O1;
    Vec3 O2;
    ComputeCentroid(P1, &Pr1, &O1);
    ComputeCentroid(P2, &Pr2, &O2);

    const Eigen::Matrix3d M = Pr2 * Pr1.transpose();
    Eigen::Matrix4d N = Eigen::Matrix4d::Zero();
    N(0, 0) = M(0, 0) + M(1, 1) + M(2, 2);
    N(0, 1) = M(1, 2) - M(2, 1);
    N(0, 2) = M(2, 0) - M(0, 2);
    N(0, 3) = M(0, 1) - M(1, 0);
    N(1, 0) = N(0, 1);
    N(1, 1) = M(0, 0) - M(1, 1) - M(2, 2);
    N(1, 2) = M(0, 1) + M(1, 0);
    N(1, 3) = M(2, 0) + M(0, 2);
    N(2, 0) = N(0, 2);
    N(2, 1) = N(1, 2);
    N(2, 2) = -M(0, 0) + M(1, 1) - M(2, 2);
    N(2, 3) = M(1, 2) + M(2, 1);
    N(3, 0) = N(0, 3);
    N(3, 1) = N(1, 3);
    N(3, 2) = N(2, 3);
    N(3, 3) = -M(0, 0) - M(1, 1) + M(2, 2);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(N);
    const Eigen::Vector4d evals = solver.eigenvalues();
    const Eigen::Matrix4d evecs = solver.eigenvectors();
    int max_index = 0;
    evals.maxCoeff(&max_index);
    // Quaternion: (w, x, y, z) from eigenvector.
    Eigen::Quaterniond q(evecs(0, max_index), evecs(1, max_index),
                         evecs(2, max_index), evecs(3, max_index));
    q.normalize();
    current_sim3_.rotation = q.toRotationMatrix();

    const Eigen::Matrix3d P3 = current_sim3_.rotation * Pr2;
    if (!fix_scale_) {
        const double nom = (Pr1.array() * P3.array()).sum();
        const double den = (P3.array() * P3.array()).sum();
        current_sim3_.scale = (den > 1e-12) ? nom / den : 1.0;
    } else {
        current_sim3_.scale = 1.0;
    }
    current_sim3_.translation =
        O1 - current_sim3_.scale * current_sim3_.rotation * O2;
}

void Sim3Solver::CheckInliers() {
    current_inliers_count_ = 0;
    current_inliers_.assign(num_matches_, false);
    for (int i = 0; i < num_matches_; ++i) {
        const Vec3 mapped = current_sim3_.Map(points_camera2_[i]);
        const double err = (mapped - points_camera1_[i]).norm();
        if (err < inlier_threshold_) {
            current_inliers_[i] = true;
            ++current_inliers_count_;
        }
    }
}

Sim3 Sim3Solver::Find(std::vector<bool>* inliers12, int* num_inliers) {
    if (inliers12 != nullptr) {
        inliers12->assign(best_inliers_.size(), false);
    }
    if (num_inliers != nullptr) {
        *num_inliers = 0;
    }
    if (num_matches_ < ransac_min_inliers_) {
        return Sim3::Identity();
    }

    std::mt19937 rng(42);
    int iterations = 0;
    const int max_its = ransac_max_iterations_;

    while (iterations < max_its) {
        ++iterations;
        if (num_matches_ < 3) {
            break;
        }
        std::vector<int> available(num_matches_);
        for (int i = 0; i < num_matches_; ++i) {
            available[i] = i;
        }
        Eigen::Matrix3d P1;
        Eigen::Matrix3d P2;
        for (int k = 0; k < 3; ++k) {
            std::uniform_int_distribution<int> dist(
                0, static_cast<int>(available.size()) - 1);
            const int r = dist(rng);
            const int idx = available[r];
            P1.col(k) = points_camera1_[idx];
            P2.col(k) = points_camera2_[idx];
            available[r] = available.back();
            available.pop_back();
        }
        ComputeSim3(P1, P2);
        CheckInliers();
        if (current_inliers_count_ > best_inliers_count_) {
            best_inliers_count_ = current_inliers_count_;
            best_sim3_ = current_sim3_;
            best_inliers_.assign(best_inliers_.size(), false);
            for (int i = 0; i < num_matches_; ++i) {
                if (current_inliers_[i]) {
                    best_inliers_[indices1_[i]] = true;
                }
            }
            if (best_inliers_count_ >= ransac_min_inliers_ &&
                best_inliers_count_ > 0.8 * num_matches_) {
                break;
            }
        }
    }

    if (inliers12 != nullptr) {
        *inliers12 = best_inliers_;
    }
    if (num_inliers != nullptr) {
        *num_inliers = best_inliers_count_;
    }
    return best_inliers_count_ >= ransac_min_inliers_ ? best_sim3_
                                                      : Sim3::Identity();
}

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
