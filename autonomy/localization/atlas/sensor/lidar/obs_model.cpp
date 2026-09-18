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

#include "autonomy/localization/atlas/sensor/lidar/obs_model.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autonomy::localization::atlas {
namespace sensor {
namespace {

bool FitPlanePca(const std::vector<Vec3_t>& pts, Vec3_t* normal, double* d) {
    if (!normal || !d || pts.size() < 3) {
        return false;
    }
    Vec3_t mean = Vec3_t::Zero();
    for (const auto& p : pts) {
        mean += p;
    }
    mean /= static_cast<double>(pts.size());
    Mat33_t cov = Mat33_t::Zero();
    for (const auto& p : pts) {
        const Vec3_t e = p - mean;
        cov += e * e.transpose();
    }
    cov /= static_cast<double>(pts.size());
    Eigen::SelfAdjointEigenSolver<Mat33_t> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    *normal = solver.eigenvectors().col(0);
    if (normal->norm() < 1e-9) {
        return false;
    }
    normal->normalize();
    // Prefer upward normal.
    if (normal->z() < 0.0) {
        *normal = -(*normal);
    }
    *d = -normal->dot(mean);
    return true;
}

}  // namespace

estimate::LidarFactorBatch ObsModel::Build(
    const std::vector<Vec3_t>& points_body,
    const std::vector<Vec3_t>& normals_world,
    const std::vector<double>& plane_d) const {
    estimate::LidarFactorBatch batch;
    const std::size_t n = std::min(
        {points_body.size(), normals_world.size(), plane_d.size(),
         static_cast<std::size_t>(std::max(0, options_.max_residuals))});
    batch.point_planes.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        estimate::PointPlaneResidual r;
        r.point_body = points_body[i];
        r.normal_world = normals_world[i];
        if (r.normal_world.norm() > 1e-9) {
            r.normal_world.normalize();
        }
        r.d = plane_d[i];
        r.weight = options_.plane_weight;
        batch.point_planes.push_back(r);
    }
    return batch;
}

estimate::LidarFactorBatch ObsModel::BuildAgainstIVox(
    const Mat44_t& T_wc,
    const std::vector<Vec3_t>& points_body,
    const mapping::IVox& map) const {
    estimate::LidarFactorBatch batch;
    if (points_body.empty()) {
        return batch;
    }
    const Mat33_t R_wc = T_wc.block<3, 3>(0, 0);
    const Vec3_t t_wc = T_wc.block<3, 1>(0, 3);
    const std::size_t limit =
        static_cast<std::size_t>(std::max(0, options_.max_residuals));
    batch.point_planes.reserve(std::min(points_body.size(), limit));
    if (options_.enable_icp_part) {
        batch.point_points.reserve(std::min(points_body.size(), limit));
    }

    for (const auto& p_b : points_body) {
        if (batch.point_planes.size() >= limit &&
            (!options_.enable_icp_part ||
             batch.point_points.size() >= limit)) {
            break;
        }
        if (!p_b.allFinite()) {
            continue;
        }
        const Vec3_t p_w = R_wc * p_b + t_wc;
        const double range2 = p_b.squaredNorm();

        // Nearest for optional P2P (also used when plane fit fails).
        std::vector<Vec3_t> nearest;
        const bool have_nn = map.GetClosestPoints(
            p_w, &nearest, map.options().knn_max_num,
            map.options().knn_max_range);

        bool plane_ok = false;
        if (batch.point_planes.size() < limit) {
            const mapping::IVox::PlaneHit hit = map.EstimatePlane(p_w);
            if (hit.ok && std::abs(hit.residual) <= options_.max_distance) {
                // lightning srange gate: reject large residual at short range.
                const double pd2 = hit.residual * hit.residual;
                if (range2 > options_.srange_scale * pd2) {
                    estimate::PointPlaneResidual r;
                    r.point_body = p_b;
                    r.normal_world = hit.normal;
                    r.d = hit.d;
                    const double scale =
                        1.0 /
                        (1.0 + std::abs(hit.residual) / options_.max_distance);
                    r.weight = options_.plane_weight * scale;
                    batch.point_planes.push_back(r);
                    plane_ok = true;
                }
            }
        }

        // lightning: ICP only on points that already passed surface selection.
        if (options_.enable_icp_part && plane_ok && have_nn &&
            !nearest.empty() && batch.point_points.size() < limit) {
            const Vec3_t e = p_w - nearest[0];
            if (e.norm() <= options_.icp_max_distance) {
                estimate::PointPointResidual r;
                r.point_body = p_b;
                r.point_world_map = nearest[0];
                r.weight = options_.icp_weight;
                batch.point_points.push_back(r);
            }
        }
    }
    return batch;
}

estimate::LidarFactorBatch ObsModel::BuildStub(
    const std::vector<Vec3_t>& points_body) const {
    estimate::LidarFactorBatch batch;
    if (!options_.enable_ground_prior || points_body.empty()) {
        return batch;
    }

    // Fit ground plane from lowest 20% z points via PCA (body frame ≈ world
    // for early frames / LO bootstrap). Fallback: Z=0 if PCA fails.
    std::vector<std::size_t> order(points_body.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return points_body[a].z() < points_body[b].z();
    });
    const std::size_t n_low = std::max<std::size_t>(
        3, static_cast<std::size_t>(0.2 * static_cast<double>(order.size())));
    std::vector<Vec3_t> low_pts;
    low_pts.reserve(n_low);
    for (std::size_t i = 0; i < n_low; ++i) {
        const auto& p = points_body[order[i]];
        if (p.allFinite()) {
            low_pts.push_back(p);
        }
    }

    Vec3_t n = Vec3_t::UnitZ();
    double d = 0.0;
    if (!FitPlanePca(low_pts, &n, &d)) {
        n = Vec3_t::UnitZ();
        d = 0.0;
    }

    const std::size_t n_out = std::min(
        points_body.size(),
        static_cast<std::size_t>(std::max(0, options_.max_residuals)));
    batch.point_planes.reserve(n_out);
    for (std::size_t i = 0; i < n_out; ++i) {
        estimate::PointPlaneResidual r;
        r.point_body = points_body[i];
        r.normal_world = n;
        r.d = d;
        r.weight = options_.ground_weight;
        batch.point_planes.push_back(r);
    }
    return batch;
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
