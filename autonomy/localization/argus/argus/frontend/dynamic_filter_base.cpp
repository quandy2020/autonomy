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

#include "argus/frontend/dynamic_filter_base.hpp"

#include <cstdint>

#include "argus/frontend/semantic_dynamic_filter.hpp"
#include "argus/frontend/spatiotemporal_dynamic_filter.hpp"

namespace argus {
namespace frontend {

void DynamicFilterBase::ApplyLidarMask(const Mask& mask,
                                       sensor::PointCloudXYZI* cloud) {
    if (cloud == nullptr || mask.keep.size() != cloud->size()) {
        return;
    }
    sensor::PointCloudXYZI filtered;
    filtered.reserve(cloud->size());
    for (std::size_t i = 0; i < cloud->size(); ++i) {
        if (mask.keep[i] != 0) {
            filtered.push_back((*cloud)[i]);
        }
    }
    cloud->swap(filtered);
}

void DynamicFilterBase::ApplyVisualMask(const Mask& mask, cv::Mat* image) {
    if (image == nullptr || image->empty()) {
        return;
    }
    const std::size_t n = static_cast<std::size_t>(image->rows * image->cols);
    if (mask.keep.size() != n) {
        return;
    }
    if (image->type() == CV_8UC1) {
        for (std::size_t i = 0; i < n; ++i) {
            if (mask.keep[i] == 0) {
                image->data[i] = 0;
            }
        }
        return;
    }
    if (image->type() == CV_8UC3) {
        for (std::size_t i = 0; i < n; ++i) {
            if (mask.keep[i] == 0) {
                uint8_t* p = image->data + 3 * i;
                p[0] = p[1] = p[2] = 0;
            }
        }
    }
}

Mask PassThroughDynamicFilter::FilterLidar(const sensor::PointCloudXYZI& cloud,
                                           const StatesGroup& /*state*/) {
    Mask m;
    m.keep.assign(cloud.size(), 1);
    return m;
}

Mask PassThroughDynamicFilter::FilterVisual(const cv::Mat& image,
                                            const StatesGroup& /*state*/) {
    Mask m;
    if (!image.empty()) {
        m.keep.assign(static_cast<std::size_t>(image.rows * image.cols), 1);
    }
    return m;
}

std::unique_ptr<DynamicFilterBase> CreateLidarDynamicFilter(
    const DynamicFilterFactoryConfig& config) {
    switch (config.lidar) {
        case DynamicFilterType::kSpatiotemporal: {
            SpatiotemporalDynamicFilterConfig st_config;
            st_config.range_diff_m = config.spatiotemporal_range_diff_m;
            st_config.azimuth_res_deg =
                config.spatiotemporal_azimuth_res_deg;
            st_config.elevation_res_deg =
                config.spatiotemporal_elevation_res_deg;
            st_config.min_range_m = config.spatiotemporal_min_range_m;
            st_config.max_range_m = config.spatiotemporal_max_range_m;
            return std::make_unique<SpatiotemporalDynamicFilter>(st_config);
        }
        case DynamicFilterType::kSemantic: {
            SemanticDynamicFilterConfig semantic_config;
            semantic_config.model_path = config.semantic_model;
            semantic_config.mask_path = config.semantic_mask_path;
            semantic_config.backend =
                ParseSemanticBackend(config.semantic_backend);
            semantic_config.mask_threshold = config.mask_threshold;
            semantic_config.input_width = config.semantic_input_width;
            semantic_config.input_height = config.semantic_input_height;
            if (!config.semantic_dynamic_class_ids.empty()) {
                semantic_config.dynamic_class_ids =
                    config.semantic_dynamic_class_ids;
            }
            return std::make_unique<SemanticDynamicFilter>(semantic_config);
        }
        case DynamicFilterType::kNone:
        default:
            return std::make_unique<PassThroughDynamicFilter>();
    }
}

}  // namespace frontend
}  // namespace argus
