/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/strata/render/slam_image_decoder.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace render {

namespace {
using costmap_2d::utils::OCC_GRID_FREE;
using costmap_2d::utils::OCC_GRID_OCCUPIED;
using costmap_2d::utils::OCC_GRID_UNKNOWN;
}  // namespace

std::vector<int16_t> DecodeSlamImageToOccupancy(const std::vector<uint8_t>& image_bytes,
                                                const SlamImageDecodeOptions& options) {
    if (image_bytes.empty() || options.width <= 0 || options.height <= 0) {
        return {};
    }

    const cv::Mat encoded(1, static_cast<int>(image_bytes.size()), CV_8UC1,
                          const_cast<uint8_t*>(image_bytes.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        return {};
    }

    cv::Mat resized;
    if (image.cols != options.width || image.rows != options.height) {
        cv::resize(image, resized, cv::Size(options.width, options.height), 0, 0, cv::INTER_AREA);
    } else {
        resized = image;
    }

    std::vector<int16_t> occupancy(static_cast<size_t>(options.width * options.height),
                                   OCC_GRID_UNKNOWN);
    for (int y = 0; y < options.height; ++y) {
        for (int x = 0; x < options.width; ++x) {
            const uint8_t pixel = resized.at<uint8_t>(y, x);
            int16_t value = OCC_GRID_UNKNOWN;
            if (pixel >= static_cast<uint8_t>(options.free_threshold)) {
                value = OCC_GRID_FREE;
            } else if (pixel <= static_cast<uint8_t>(options.occupied_threshold)) {
                value = OCC_GRID_OCCUPIED;
            }
            occupancy[static_cast<size_t>(y * options.width + x)] = value;
        }
    }
    return occupancy;
}

}  // namespace render
}  // namespace strata
}  // namespace map
}  // namespace autonomy
