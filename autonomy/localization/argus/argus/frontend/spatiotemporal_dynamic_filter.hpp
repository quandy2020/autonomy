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
 * @file spatiotemporal_dynamic_filter.hpp
 * @brief Scan-to-scan spherical range inconsistency (no learned model).
 */

#ifndef ARGUS_FRONTEND_SPATIOTEMPORAL_DYNAMIC_FILTER_HPP_
#define ARGUS_FRONTEND_SPATIOTEMPORAL_DYNAMIC_FILTER_HPP_

#include <vector>

#include "argus/frontend/dynamic_filter_base.hpp"

namespace argus {
namespace frontend {

/**
 * @brief Parameters for spherical-bin range differencing.
 */
struct SpatiotemporalDynamicFilterConfig {
    double range_diff_m =
        0.4; /**< @brief Min closer range to mark dynamic (m). */
    double azimuth_res_deg = 0.4;   /**< @brief Azimuth bin width (deg). */
    double elevation_res_deg = 0.4; /**< @brief Elevation bin width (deg). */
    double min_range_m = 1.0; /**< @brief Ignore points closer than this (m). */
    double max_range_m =
        80.0; /**< @brief Ignore points farther than this (m). */
    double elev_min_deg = -25.0; /**< @brief Elevation FOV lower bound (deg). */
    double elev_max_deg = 15.0;  /**< @brief Elevation FOV upper bound (deg). */
};

/**
 * @brief Marks points much closer than the previous scan at the same bearing
 *        as dynamic (object entered the ray). First frame keeps all.
 */
class SpatiotemporalDynamicFilter : public DynamicFilterBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(SpatiotemporalDynamicFilter);

    /**
     * @brief Construct with spherical-bin parameters.
     * @param config Range / FOV / resolution settings.
     */
    explicit SpatiotemporalDynamicFilter(
        SpatiotemporalDynamicFilterConfig config = {});

    /**
     * @brief Compare current cloud to previous spherical range image.
     * @param cloud Body-frame LiDAR points.
     * @param state Unused (API parity with other filters).
     * @return Keep mask (1 = static).
     */
    Mask FilterLidar(const sensor::PointCloudXYZI& cloud,
                     const StatesGroup& state) override;

    /**
     * @brief Visual path is pass-through (no image model).
     * @param image Input image (unused for keep decisions).
     * @param state Unused.
     * @return All-keep mask sized to image pixels.
     */
    Mask FilterVisual(const cv::Mat& image, const StatesGroup& state) override;

    /** @brief Drop previous-scan range image (next FilterLidar re-inits). */
    void Reset();

private:
    struct SphericalIndex {
        int az = 0;
        int el = 0;
        bool valid = false;
    };

    SphericalIndex Project(const Eigen::Vector3d& p_sensor) const;
    int FlatIndex(int az, int el) const {
        return el * az_bins_ + az;
    }

    SpatiotemporalDynamicFilterConfig config_;
    int az_bins_ = 1;
    int el_bins_ = 1;
    std::vector<float> prev_range_;
    std::vector<uint8_t> prev_valid_;
    bool has_prev_ = false;
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_SPATIOTEMPORAL_DYNAMIC_FILTER_HPP_
