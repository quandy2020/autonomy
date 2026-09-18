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

#include "autonomy/localization/atlas/frontend/lidar_loc/lidar_locator.hpp"

#include <algorithm>
#include <cmath>

#include "glog/logging.h"

#if __has_include(<pcl/registration/ndt.h>)
#define ATLAS_LIDAR_LOC_HAS_NDT 1
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#else
#define ATLAS_LIDAR_LOC_HAS_NDT 0
#endif

namespace autonomy::localization::atlas {
namespace frontend {
namespace {

#if ATLAS_LIDAR_LOC_HAS_NDT
using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

PclCloud::Ptr ToPcl(const std::vector<Vec3_t>& pts) {
    auto cloud = std::make_shared<PclCloud>();
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        if (!p.allFinite()) {
            continue;
        }
        cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x()),
                                       static_cast<float>(p.y()),
                                       static_cast<float>(p.z())));
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

PclCloud::Ptr VoxelDown(const PclCloud::Ptr& in, float leaf) {
    if (!in || in->empty()) {
        return in;
    }
    leaf = std::max(leaf, 0.05f);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    auto out = std::make_shared<PclCloud>();
    vg.filter(*out);
    return out;
}
#endif

}  // namespace

bool LidarLocator::LoadMap(const std::string& dir) {
    if (dir.empty()) {
        return false;
    }
    if (!map_) {
        owned_map_ = std::make_unique<mapping::TiledMap>();
        map_ = owned_map_.get();
    }
    if (!map_->Load(dir)) {
        LOG(WARNING) << "LidarLocator: failed to load tiled map from " << dir;
        return false;
    }
    LOG(INFO) << "LidarLocator: loaded tiled map " << dir
              << " tiles=" << map_->num_tiles();
    return true;
}

bool LidarLocator::Align(const std::vector<Vec3_t>& points_body,
                         const Mat44_t& T_wb_guess, Mat44_t* T_wb_out,
                         double* score_out) {
    if (!T_wb_out || !map_ || points_body.size() < options_.min_scan_points) {
        return false;
    }
#if !ATLAS_LIDAR_LOC_HAS_NDT
    (void)T_wb_guess;
    (void)score_out;
    LOG(WARNING) << "LidarLocator: PCL NDT unavailable";
    return false;
#else
    const Vec3_t pos = T_wb_guess.block<3, 1>(0, 3);
    map_->LoadOnPose(pos, options_.load_radius_m);

    std::vector<Vec3_t> map_pts;
    map_pts.reserve(50000);
    for (const auto& kv : map_->tiles()) {
        const auto& tile = kv.second;
        if (!tile.loaded) {
            continue;
        }
        map_pts.insert(map_pts.end(), tile.points.begin(), tile.points.end());
    }
    if (map_pts.size() < options_.min_map_points) {
        return false;
    }

    auto scan = VoxelDown(ToPcl(points_body), options_.voxel_leaf);
    auto target = VoxelDown(ToPcl(map_pts), options_.voxel_leaf);
    if (!scan || !target || scan->size() < options_.min_scan_points ||
        target->size() < options_.min_map_points) {
        return false;
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(options_.ndt_resolution);
    ndt.setStepSize(options_.ndt_step_size);
    ndt.setTransformationEpsilon(options_.ndt_trans_eps);
    ndt.setMaximumIterations(options_.ndt_max_iter);
    ndt.setInputSource(scan);
    ndt.setInputTarget(target);

    Eigen::Matrix4f guess = T_wb_guess.cast<float>();
    PclCloud aligned;
    ndt.align(aligned, guess);
    if (!ndt.hasConverged()) {
        return false;
    }
    const double score = static_cast<double>(ndt.getFitnessScore());
    if (score_out) {
        *score_out = score;
    }
    if (options_.min_score > 0.0 && score > options_.min_score) {
        // PCL fitness = mean squared distance; lower is better.
        return false;
    }
    *T_wb_out = ndt.getFinalTransformation().cast<double>();
    return true;
#endif
}

}  // namespace frontend
}  // namespace autonomy::localization::atlas
