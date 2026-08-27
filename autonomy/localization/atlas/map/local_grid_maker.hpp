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

#pragma once

#include <opencv2/core/mat.hpp>

#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/map/local_grid.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * Build a LocalGrid from one RGB-D observation (RTAB-Map LocalGridMaker style).
 *
 * Pipeline:
 *   depth → optical → camera_link → footprint crop → height window
 *   → height-band OR normals segmentation → voxelize → noise filter
 *   → optional ray-fill empty.
 */
class LocalGridMaker {
public:
    struct Options {
        float cell_size = 0.05f;
        float range_min = 0.2f;
        float range_max = 5.0f;
        int depth_decimation = 4;
        /** Heights in camera_link (Z up). Autosim camera ~0.6 m → ground ≈ -0.6. */
        float min_ground_height = -0.85f;
        float max_ground_height = -0.25f;
        float max_obstacle_height = 1.5f;
        /** Robot body box in camera_link (m); 0 disables. RTAB-Map Grid/Footprint*. */
        float footprint_length = 0.35f;
        float footprint_width = 0.35f;
        float footprint_height = 0.5f;
        /** Drop voxels with fewer than N neighbors in 26-neighborhood (0=off). */
        int noise_filtering_radius = 1;
        int noise_filtering_min_neighbors = 2;
        /** Treat ground cells as obstacles (UAV / no traversable floor). */
        bool ground_is_obstacle = false;
        /**
         * RTAB-Map Grid/NormalsSegmentation: estimate normals + cluster
         * flat patches as ground. Falls back to Z height-band when false.
         */
        bool normals_segmentation = false;
        int normal_k = 20;
        float max_ground_angle_deg = 45.f;
        float cluster_radius = 0.1f;
        int min_cluster_size = 10;
        /** Elevated flat patches become obstacles (RTAB-Map FlatObstacleDetected). */
        bool flat_obstacles = true;
        bool ray_tracing = true;
        bool store_rgb = true;
        int max_cells_per_layer = 200000;
    };

    explicit LocalGridMaker(Options options);

    LocalGrid Create(const cv::Mat& rgb, const cv::Mat& depth_m,
                     const Mat44_t& T_wc_opencv, camera::base* camera,
                     double timestamp_sec, uint64_t id) const;

    const Options& options() const { return options_; }

private:
    Options options_;
};

/** OpenCV optical (X right, Y down, Z fwd) → ROS camera_link FLU. */
Mat33_t OpticalToLinkR();
Mat44_t OpencvPoseToRosCameraLink(const Mat44_t& T_wc_opencv);
Mat44_t OpencvPoseToRosMapOptical(const Mat44_t& T_wc_opencv);

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
