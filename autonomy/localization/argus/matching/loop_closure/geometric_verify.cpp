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

#include "argus/relocalization/loop_closure/geometric_verify.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "argus/common/math/transform/transform.hpp"

namespace argus {
namespace loop_closure {
namespace {

struct Corr {
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
};

std::vector<Eigen::Vector3d> SampleCloud(const sensor::PointCloudXYZI& cloud,
                                         int stride) {
    std::vector<Eigen::Vector3d> out;
    const int s = std::max(1, stride);
    out.reserve(cloud.size() / static_cast<std::size_t>(s) + 1);
    for (std::size_t i = 0; i < cloud.size(); i += static_cast<std::size_t>(s)) {
        const auto& p = cloud.points[i];
        out.emplace_back(p.x, p.y, p.z);
    }
    return out;
}

int FindNearest(const std::vector<Eigen::Vector3d>& cloud,
                const Eigen::Vector3d& q, double* dist2) {
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const double d2 = (cloud[i] - q).squaredNorm();
        if (d2 < best_d2) {
            best_d2 = d2;
            best = static_cast<int>(i);
        }
    }
    if (dist2 != nullptr) {
        *dist2 = best_d2;
    }
    return best;
}

bool Kabsch(const std::vector<Corr>& corrs, transform::Rigid3d* T) {
    if (T == nullptr || corrs.size() < 3) {
        return false;
    }
    Eigen::Vector3d mu_q = Eigen::Vector3d::Zero();
    Eigen::Vector3d mu_m = Eigen::Vector3d::Zero();
    for (const Corr& c : corrs) {
        mu_q += c.q;
        mu_m += c.m;
    }
    const double inv = 1.0 / static_cast<double>(corrs.size());
    mu_q *= inv;
    mu_m *= inv;

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (const Corr& c : corrs) {
        W += (c.q - mu_q) * (c.m - mu_m).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0.0) {
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1.0;
        R = U * svd.matrixV().transpose();
    }
    const Eigen::Vector3d t = mu_q - R * mu_m;
    *T = transform::Rigid3d(t, Eigen::Quaterniond(R));
    return true;
}

GeometricVerifyResult RunIcpOnce(const std::vector<Eigen::Vector3d>& q_pts,
                                 const std::vector<Eigen::Vector3d>& m_pts,
                                 const transform::Rigid3d& T_seed,
                                 const GeometricVerifyConfig& config) {
    GeometricVerifyResult result;
    result.T_query_match = T_seed;
    transform::Rigid3d T = T_seed;
    double fitness = 1e9;
    int n_corr = 0;
    const double final_corr = std::max(0.2, config.max_corr_dist_m);
    const double coarse_corr = std::max(final_corr, 2.0 * final_corr);

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        // Coarse → fine correspondence radius.
        const double alpha =
            static_cast<double>(iter) /
            static_cast<double>(std::max(1, config.max_iterations - 1));
        const double corr_r = coarse_corr * (1.0 - alpha) + final_corr * alpha;
        const double max_corr2 = corr_r * corr_r;

        std::vector<Corr> corrs;
        corrs.reserve(m_pts.size());
        double sum_d = 0.0;
        for (const Eigen::Vector3d& m : m_pts) {
            const Eigen::Vector3d pred = T * m;
            double d2 = 0.0;
            const int j = FindNearest(q_pts, pred, &d2);
            if (j < 0 || d2 > max_corr2) {
                continue;
            }
            Corr c;
            c.q = q_pts[static_cast<std::size_t>(j)];
            c.m = m;
            corrs.push_back(c);
            sum_d += std::sqrt(d2);
        }
        n_corr = static_cast<int>(corrs.size());
        if (n_corr < config.min_correspondences) {
            result.correspondences = n_corr;
            result.inlier_ratio =
                static_cast<double>(n_corr) /
                static_cast<double>(std::max<std::size_t>(1, m_pts.size()));
            result.fail_reason = GeomFailReason::kTooFewCorrespondences;
            return result;
        }
        fitness = sum_d / static_cast<double>(n_corr);
        transform::Rigid3d T_new;
        if (!Kabsch(corrs, &T_new)) {
            result.fail_reason = GeomFailReason::kTooFewCorrespondences;
            return result;
        }
        const double dT =
            (T.inverse() * T_new).translation().norm() +
            std::abs(transform::GetYaw(T.inverse() * T_new));
        T = T_new;
        if (dT < 1e-3) {
            break;
        }
    }

    result.T_query_match = T;
    result.fitness_m = fitness;
    result.correspondences = n_corr;
    result.inlier_ratio =
        static_cast<double>(n_corr) /
        static_cast<double>(std::max<std::size_t>(1, m_pts.size()));
    const double yaw = std::abs(transform::GetYaw(T));
    const double trans = T.translation().norm();
    if (n_corr < config.min_correspondences) {
        result.fail_reason = GeomFailReason::kTooFewCorrespondences;
    } else if (fitness > config.max_fitness_m) {
        result.fail_reason = GeomFailReason::kFitness;
    } else if (trans > config.max_translation_m) {
        result.fail_reason = GeomFailReason::kTranslation;
    } else if (yaw > config.max_yaw_rad) {
        result.fail_reason = GeomFailReason::kYaw;
    } else if (config.min_inlier_ratio > 0.0 &&
               result.inlier_ratio < config.min_inlier_ratio) {
        result.fail_reason = GeomFailReason::kInlierRatio;
    } else {
        result.ok = true;
        result.fail_reason = GeomFailReason::kOk;
    }
    return result;
}

std::vector<transform::Rigid3d> BuildSeedLattice(
    const transform::Rigid3d& T_seed, const GeometricVerifyConfig& config) {
    std::vector<transform::Rigid3d> seeds;
    const int nxy = std::max(0, config.seed_xy_steps);
    const int nyaw = std::max(0, config.seed_yaw_steps);
    const double dxy = std::max(0.1, config.seed_xy_step_m);
    const double dyaw = std::max(0.05, config.seed_yaw_step_rad);
    seeds.reserve(static_cast<std::size_t>((2 * nxy + 1) * (2 * nxy + 1) *
                                           (2 * nyaw + 1)));
    for (int ix = -nxy; ix <= nxy; ++ix) {
        for (int iy = -nxy; iy <= nxy; ++iy) {
            for (int iyaw = -nyaw; iyaw <= nyaw; ++iyaw) {
                const transform::Rigid3d dT = transform::FromXyzRpy(
                    ix * dxy, iy * dxy, 0.0, 0.0, 0.0, iyaw * dyaw);
                seeds.push_back(dT * T_seed);
            }
        }
    }
    if (seeds.empty()) {
        seeds.push_back(T_seed);
    }
    return seeds;
}

}  // namespace

GeometricVerifyResult VerifyAndRefineLoop(
    const sensor::PointCloudXYZI& cloud_query,
    const sensor::PointCloudXYZI& cloud_match,
    const transform::Rigid3d& T_seed, const GeometricVerifyConfig& config) {
    GeometricVerifyResult result;
    result.T_query_match = T_seed;
    if (!config.enable) {
        result.ok = true;
        result.fail_reason = GeomFailReason::kOk;
        result.fitness_m = 0.0;
        return result;
    }
    if (cloud_query.empty() || cloud_match.empty()) {
        result.fail_reason = GeomFailReason::kEmptyCloud;
        return result;
    }

    const auto q_pts = SampleCloud(cloud_query, config.subsample_stride);
    const auto m_pts = SampleCloud(cloud_match, config.subsample_stride);
    if (static_cast<int>(q_pts.size()) < config.min_correspondences ||
        static_cast<int>(m_pts.size()) < config.min_correspondences) {
        result.fail_reason = GeomFailReason::kTooFewPoints;
        return result;
    }

    GeometricVerifyResult best;
    best.fitness_m = 1e9;
    best.fail_reason = GeomFailReason::kFitness;
    for (const transform::Rigid3d& seed : BuildSeedLattice(T_seed, config)) {
        GeometricVerifyResult trial = RunIcpOnce(q_pts, m_pts, seed, config);
        if (trial.ok &&
            (best.ok ? trial.fitness_m < best.fitness_m : true)) {
            best = trial;
        } else if (!best.ok && trial.fitness_m < best.fitness_m) {
            best = trial;  // keep diagnostics from closest failure
        }
    }
    return best;
}

}  // namespace loop_closure
}  // namespace argus
