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

#include "autonomy/localization/atlas/sensor/lidar/lightning/obs_model/obs_model.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::localization::atlas {
namespace sensor {
namespace lightning {

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
        r.weight = 1.0;
        batch.point_planes.push_back(r);
    }
    return batch;
}

estimate::LidarFactorBatch ObsModel::BuildAgainstIVox(
    const Mat44_t& T_wc,
    const std::vector<Vec3_t>& points_body,
    const IVox& map) const {
    estimate::LidarFactorBatch batch;
    if (points_body.empty()) {
        return batch;
    }
    const Mat33_t R_wc = T_wc.block<3, 3>(0, 0);
    const Vec3_t t_wc = T_wc.block<3, 1>(0, 3);
    const std::size_t limit =
        static_cast<std::size_t>(std::max(0, options_.max_residuals));
    batch.point_planes.reserve(std::min(points_body.size(), limit));

    for (const auto& p_b : points_body) {
        if (batch.point_planes.size() >= limit) {
            break;
        }
        if (!p_b.allFinite()) {
            continue;
        }
        const Vec3_t p_w = R_wc * p_b + t_wc;
        const IVox::PlaneHit hit = map.EstimatePlane(p_w);
        if (!hit.ok) {
            continue;
        }
        if (std::abs(hit.residual) > options_.max_distance) {
            continue;
        }
        estimate::PointPlaneResidual r;
        r.point_body = p_b;
        r.normal_world = hit.normal;
        r.d = hit.d;
        // Down-weight large residuals.
        const double scale =
            1.0 / (1.0 + std::abs(hit.residual) / options_.max_distance);
        r.weight = scale;
        batch.point_planes.push_back(r);
    }
    return batch;
}

estimate::LidarFactorBatch ObsModel::BuildStub(
    const std::vector<Vec3_t>& points_body) const {
    estimate::LidarFactorBatch batch;
    if (!options_.enable_ground_prior || points_body.empty()) {
        return batch;
    }
    const std::size_t n = std::min(
        points_body.size(),
        static_cast<std::size_t>(std::max(0, options_.max_residuals)));
    batch.point_planes.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        estimate::PointPlaneResidual r;
        r.point_body = points_body[i];
        r.normal_world = Vec3_t::UnitZ();
        r.d = 0.0;
        r.weight = options_.ground_weight;
        batch.point_planes.push_back(r);
    }
    return batch;
}

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas
