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
 * @file dynamic_filter_base.hpp
 * @brief Front-end dynamic object masks (before residual / ESIKF).
 */

#ifndef ARGUS_FRONTEND_DYNAMIC_FILTER_BASE_HPP_
#define ARGUS_FRONTEND_DYNAMIC_FILTER_BASE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "argus/common/macros.hpp"
#include "argus/frontend/dynamic_filter_factory_config.hpp"
#include "argus/frontend/types.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace frontend {

/** @brief Per-element keep flags (1 = static / retain). */
struct Mask {
    std::vector<uint8_t> keep;

    std::size_t size() const {
        return keep.size();
    }
    bool empty() const {
        return keep.empty();
    }

    /** @brief Fraction of entries with keep==0; 0 if empty. */
    double RejectedRatio() const {
        if (keep.empty()) {
            return 0.0;
        }
        std::size_t rejected = 0;
        for (uint8_t v : keep) {
            if (v == 0) {
                ++rejected;
            }
        }
        return static_cast<double>(rejected) / static_cast<double>(keep.size());
    }
};

/**
 * @brief Abstract dynamic-object filter applied before ESIKF / VIO.
 */
class DynamicFilterBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(DynamicFilterBase);

    virtual ~DynamicFilterBase() = default;

    /**
     * @brief Compute per-point keep flags for a LiDAR sweep.
     * @param cloud Body-frame points.
     * @param state Current ESIKF state (may be unused by some backends).
     * @return Mask with `keep.size() == cloud.size()` (1 = static).
     */
    virtual Mask FilterLidar(const sensor::PointCloudXYZI& cloud,
                             const StatesGroup& state) = 0;

    /**
     * @brief Compute per-pixel keep flags for an image.
     * @param image BGR or gray frame.
     * @param state Current ESIKF state (may be unused).
     * @return Mask sized to `rows * cols` (1 = static).
     */
    virtual Mask FilterVisual(const cv::Mat& image,
                              const StatesGroup& state) = 0;

    /**
     * @brief Compact cloud in-place using keep flags (same length).
     * @param mask Keep mask from FilterLidar.
     * @param[in,out] cloud Cloud to filter; resized to survivors.
     */
    static void ApplyLidarMask(const Mask& mask, sensor::PointCloudXYZI* cloud);

    /**
     * @brief Zero BGR/gray pixels where keep==0 (same length = rows*cols).
     *        Leaves geometry intact for VIO patch sampling.
     * @param mask Keep mask from FilterVisual.
     * @param[in,out] image Image whose rejected pixels are cleared.
     */
    static void ApplyVisualMask(const Mask& mask, cv::Mat* image);
};

/** @brief Keep everything (config: none). */
class PassThroughDynamicFilter : public DynamicFilterBase
{
public:
    Mask FilterLidar(const sensor::PointCloudXYZI& cloud,
                     const StatesGroup& /*state*/) override;
    Mask FilterVisual(const cv::Mat& image,
                      const StatesGroup& /*state*/) override;
};

/**
 * @brief Create LiDAR filter instance from factory config.
 * @param config Backend selection and parameters.
 * @return Owning pointer (never null; may be PassThrough).
 */
std::unique_ptr<DynamicFilterBase> CreateLidarDynamicFilter(
    const DynamicFilterFactoryConfig& config);

/**
 * @brief Create visual filter (`config.visual` policy).
 * @param config Factory config; `visual` selects backend.
 * @return Owning pointer (never null).
 */
inline std::unique_ptr<DynamicFilterBase> CreateVisualDynamicFilter(
    const DynamicFilterFactoryConfig& config) {
    DynamicFilterFactoryConfig visual_config = config;
    visual_config.lidar = config.visual;
    return CreateLidarDynamicFilter(visual_config);
}

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_DYNAMIC_FILTER_BASE_HPP_
