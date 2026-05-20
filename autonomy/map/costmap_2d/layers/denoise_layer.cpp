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

#include "autonomy/map/costmap_2d/layers/denoise_layer.hpp"

#include <cmath>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"

namespace autonomy {
namespace map {
namespace costmap_2d {

void DenoiseLayer::onInitialize() {
    enabled_ = true;
    minimal_group_size_ = 2;
    group_connectivity_type_ = ConnectivityType::Way8;

    if (options_ && options_->has_denoise_layer()) {
        const auto& denoise_opts = options_->denoise_layer();
        enabled_ = denoise_opts.enabled();

        // denoise_radius in config maps to Nav2 minimal_group_size (cells).
        const int minimal_group_size_param =
            static_cast<int>(std::lround(denoise_opts.denoise_radius()));
        if (minimal_group_size_param <= 1) {
            AWARN << "DenoiseLayer: denoise_radius=" << denoise_opts.denoise_radius()
                  << " (minimal_group_size <= 1): no denoising will be applied";
            minimal_group_size_ = 1;
        } else {
            minimal_group_size_ =
                static_cast<size_t>(minimal_group_size_param);
        }

        const int connectivity = denoise_opts.group_connectivity_type();
        if (connectivity == 4) {
            group_connectivity_type_ = ConnectivityType::Way4;
        } else if (connectivity == 8) {
            group_connectivity_type_ = ConnectivityType::Way8;
        } else if (connectivity != 0) {
            AWARN << "DenoiseLayer: group_connectivity_type=" << connectivity
                  << " invalid, using 8";
            group_connectivity_type_ = ConnectivityType::Way8;
        }
    }

    AINFO << "DenoiseLayer initialized: enabled=" << enabled_
          << ", minimal_group_size=" << minimal_group_size_
          << ", connectivity="
          << (group_connectivity_type_ == ConnectivityType::Way4 ? 4 : 8);

    current_ = true;
}

void DenoiseLayer::reset() {
    current_ = false;
}

bool DenoiseLayer::isClearable() {
    return false;
}

void DenoiseLayer::updateBounds(double /*robot_x*/, double /*robot_y*/,
                                double /*robot_yaw*/, double* /*min_x*/,
                                double* /*min_y*/, double* /*max_x*/,
                                double* /*max_y*/) {
    // Filter layer: does not expand update bounds.
}

void DenoiseLayer::updateCosts(Costmap2D& master_grid, int min_x, int min_y,
                               int max_x, int max_y) {
    if (!enabled_) {
        return;
    }

    if (min_x >= max_x || min_y >= max_y) {
        return;
    }

    if (minimal_group_size_ <= 1) {
        current_ = true;
        return;
    }

    no_information_is_obstacle_ =
        master_grid.getDefaultValue() != NO_INFORMATION;

    unsigned char* master_array = master_grid.getCharMap();
    const int step = static_cast<int>(master_grid.getSizeInCellsX());

    const size_t width = static_cast<size_t>(max_x - min_x);
    const size_t height = static_cast<size_t>(max_y - min_y);
    Image<uint8_t> roi_image(height, width, master_array + min_y * step + min_x,
                             step);

    try {
        denoise(roi_image);
    } catch (const std::exception& ex) {
        AWARN << "DenoiseLayer updateCosts failed: " << ex.what();
    }

    current_ = true;
}

void DenoiseLayer::denoise(Image<uint8_t>& image) const {
    if (image.empty()) {
        return;
    }

    if (minimal_group_size_ <= 1) {
        return;
    }

    if (minimal_group_size_ == 2) {
        removeSinglePixels(image);
    } else {
        removeGroups(image);
    }
}

void DenoiseLayer::removeGroups(Image<uint8_t>& image) const {
    groups_remover_.removeGroups(
        image, buffer_, group_connectivity_type_, minimal_group_size_,
        [this](uint8_t pixel) { return isBackground(pixel); });
}

void DenoiseLayer::removeSinglePixels(Image<uint8_t>& image) const {
    uint8_t* buf = buffer_.get<uint8_t>(image.rows() * image.columns());
    Image<uint8_t> max_neighbors_image(image.rows(), image.columns(), buf,
                                       image.columns());

    if (!no_information_is_obstacle_) {
        auto replace_to_free = [](uint8_t v) {
            return v == NO_INFORMATION ? FREE_SPACE : v;
        };
        auto max = [&](const std::initializer_list<uint8_t> lst) {
            std::array<uint8_t, 3> rbuf = {replace_to_free(*lst.begin()),
                                           replace_to_free(*(lst.begin() + 1)),
                                           replace_to_free(*(lst.begin() + 2))};
            return *std::max_element(rbuf.begin(), rbuf.end());
        };
        dilate(image, max_neighbors_image, group_connectivity_type_, max);
    } else {
        auto max = [](const std::initializer_list<uint8_t> lst) {
            return std::max(lst);
        };
        dilate(image, max_neighbors_image, group_connectivity_type_, max);
    }

    max_neighbors_image.convert(
        image, [this](uint8_t max_neighbor, uint8_t& img) {
            if (!isBackground(img) && isBackground(max_neighbor)) {
                img = FREE_SPACE;
            }
        });
}

bool DenoiseLayer::isBackground(uint8_t pixel) const {
    const bool is_obstacle =
        pixel == LETHAL_OBSTACLE || pixel == INSCRIBED_INFLATED_OBSTACLE ||
        (pixel == NO_INFORMATION && no_information_is_obstacle_);
    return !is_obstacle;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
