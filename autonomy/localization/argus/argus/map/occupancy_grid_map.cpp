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

#include "argus/map/occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace argus {
namespace map {

OccupancyGridMap::OccupancyGridMap(OccupancyGridMapOptions options)
    : options_(std::move(options)) {
    grid_reso_ = options_.resolution * static_cast<float>(kSubWidth);
}

OccupancyGridMap::~OccupancyGridMap() {
    ReleaseResources();
}

void OccupancyGridMap::ReleaseResources() {
    if (grids_ != nullptr) {
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
        grids_ = nullptr;
    }
    min_x_ = min_y_ = 10000.f;
    max_x_ = max_y_ = -10000.f;
    grid_size_x_ = grid_size_y_ = 0;
}

void OccupancyGridMap::Clear() {
    ReleaseResources();
}

bool OccupancyGridMap::Init(float temp_min_x, float temp_min_y, float temp_max_x,
                            float temp_max_y) {
    ReleaseResources();
    // Snap to coarse tile size so fine indices stay tile-aligned on Resize.
    min_x_ = std::floor(temp_min_x / grid_reso_) * grid_reso_;
    min_y_ = std::floor(temp_min_y / grid_reso_) * grid_reso_;
    max_x_ = std::ceil(temp_max_x / grid_reso_) * grid_reso_;
    max_y_ = std::ceil(temp_max_y / grid_reso_) * grid_reso_;

    grid_size_x_ =
        static_cast<int>(std::ceil((max_x_ - min_x_) / grid_reso_));
    grid_size_y_ =
        static_cast<int>(std::ceil((max_y_ - min_y_) / grid_reso_));
    if (grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        return false;
    }

    grids_ = new OccupancyGridSubgrid*[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        grids_[xi] = new OccupancyGridSubgrid[grid_size_y_];
    }
    return true;
}

OccupancyGridMap::SharedPtr OccupancyGridMap::MakeDeepCopy() const {
    auto ret = OccupancyGridMap::make_shared(options_);
    ret->min_x_ = min_x_;
    ret->min_y_ = min_y_;
    ret->max_x_ = max_x_;
    ret->max_y_ = max_y_;
    ret->grid_size_x_ = grid_size_x_;
    ret->grid_size_y_ = grid_size_y_;
    ret->grid_reso_ = grid_reso_;
    if (grids_ == nullptr) {
        return ret;
    }
    ret->grids_ = new OccupancyGridSubgrid*[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        ret->grids_[xi] = new OccupancyGridSubgrid[grid_size_y_];
        for (int yi = 0; yi < grid_size_y_; ++yi) {
            ret->grids_[xi][yi] = grids_[xi][yi];
        }
    }
    return ret;
}

bool OccupancyGridMap::Resize(float temp_min_x, float temp_min_y,
                              float temp_max_x, float temp_max_y) {
    if (grids_ == nullptr) {
        return Init(temp_min_x, temp_min_y, temp_max_x, temp_max_y);
    }

    // Align to coarse tiles so copied Subgrids keep world correspondence.
    temp_min_x = std::floor(temp_min_x / grid_reso_) * grid_reso_;
    temp_min_y = std::floor(temp_min_y / grid_reso_) * grid_reso_;
    temp_max_x = std::ceil(temp_max_x / grid_reso_) * grid_reso_;
    temp_max_y = std::ceil(temp_max_y / grid_reso_) * grid_reso_;

    const int temp_grid_size_x =
        static_cast<int>(std::ceil((temp_max_x - temp_min_x) / grid_reso_));
    const int temp_grid_size_y =
        static_cast<int>(std::ceil((temp_max_y - temp_min_y) / grid_reso_));
    if (temp_grid_size_x <= 0 || temp_grid_size_y <= 0) {
        return false;
    }

    auto** new_grids = new OccupancyGridSubgrid*[temp_grid_size_x];
    for (int xi = 0; xi < temp_grid_size_x; ++xi) {
        new_grids[xi] = new OccupancyGridSubgrid[temp_grid_size_y];
    }

    const int min_grid_x =
        static_cast<int>(std::lround((temp_min_x - min_x_) / grid_reso_));
    const int min_grid_y =
        static_cast<int>(std::lround((temp_min_y - min_y_) / grid_reso_));

    for (int x = 0; x < grid_size_x_; ++x) {
        for (int y = 0; y < grid_size_y_; ++y) {
            const int nx = x - min_grid_x;
            const int ny = y - min_grid_y;
            if (nx >= 0 && nx < temp_grid_size_x && ny >= 0 &&
                ny < temp_grid_size_y) {
                new_grids[nx][ny] = grids_[x][y];
            }
        }
        delete[] grids_[x];
    }
    delete[] grids_;

    grids_ = new_grids;
    min_x_ = temp_min_x;
    min_y_ = temp_min_y;
    max_x_ = temp_max_x;
    max_y_ = temp_max_y;
    grid_size_x_ = temp_grid_size_x;
    grid_size_y_ = temp_grid_size_y;
    return true;
}

void OccupancyGridMap::SetHitPoint(float px, float py, bool hit, float height) {
    if (grids_ == nullptr) {
        return;
    }
    if (px < min_x_ || px > max_x_ || py < min_y_ || py > max_y_) {
        return;
    }
    const int x_index =
        static_cast<int>(std::floor((px - min_x_) / options_.resolution));
    const int y_index =
        static_cast<int>(std::floor((py - min_y_) / options_.resolution));
    UpdateCell(Eigen::Vector2i(x_index, y_index), hit, height);
}

void OccupancyGridMap::SetHitPointUnlocked(float px, float py, bool hit,
                                           float height) {
    if (grids_ == nullptr) {
        return;
    }
    if (px < min_x_ || px > max_x_ || py < min_y_ || py > max_y_) {
        return;
    }
    const int x_index =
        static_cast<int>(std::floor((px - min_x_) / options_.resolution));
    const int y_index =
        static_cast<int>(std::floor((py - min_y_) / options_.resolution));
    UpdateCellUnlocked(Eigen::Vector2i(x_index, y_index), hit, height);
}

void OccupancyGridMap::UpdateCell(const Eigen::Vector2i& point_index, bool hit,
                                  float height) {
    if (grids_ == nullptr) {
        return;
    }
    const int x_index = point_index.x();
    const int y_index = point_index.y();
    const int xi = x_index >> kSubLog2;
    const int yi = y_index >> kSubLog2;
    if (xi < 0 || xi > (grid_size_x_ - 1) || yi < 0 || yi > (grid_size_y_ - 1)) {
        return;
    }
    const int sub_i = x_index - (xi << kSubLog2);
    const int sub_j = y_index - (yi << kSubLog2);
    if (sub_i < 0 || sub_i > (kSubWidth - 1) || sub_j < 0 ||
        sub_j > (kSubWidth - 1)) {
        return;
    }
    grids_[xi][yi].SetHit(hit, sub_i, sub_j, height);
}

void OccupancyGridMap::UpdateCellUnlocked(const Eigen::Vector2i& point_index,
                                          bool hit, float height) {
    if (grids_ == nullptr) {
        return;
    }
    const int x_index = point_index.x();
    const int y_index = point_index.y();
    const int xi = x_index >> kSubLog2;
    const int yi = y_index >> kSubLog2;
    if (xi < 0 || xi > (grid_size_x_ - 1) || yi < 0 || yi > (grid_size_y_ - 1)) {
        return;
    }
    const int sub_i = x_index - (xi << kSubLog2);
    const int sub_j = y_index - (yi << kSubLog2);
    if (sub_i < 0 || sub_i > (kSubWidth - 1) || sub_j < 0 ||
        sub_j > (kSubWidth - 1)) {
        return;
    }
    grids_[xi][yi].SetHitUnlocked(hit, sub_i, sub_j, height);
}

bool OccupancyGridMap::WorldToFineIndex(float x, float y, int* x_index,
                                        int* y_index) const {
    if (x > max_x_ || x < min_x_ || y > max_y_ || y < min_y_) {
        return false;
    }
    *x_index =
        static_cast<int>(std::floor((x - min_x_) / options_.resolution));
    *y_index =
        static_cast<int>(std::floor((y - min_y_) / options_.resolution));
    return true;
}

void OccupancyGridMap::SetMissPoint(float point_x, float point_y,
                                    float origin_x, float origin_y, float height,
                                    float lidar_height) {
    SetMissPointImpl(point_x, point_y, origin_x, origin_y, height, lidar_height,
                     /*unlocked=*/false);
}

void OccupancyGridMap::SetMissPointUnlocked(float point_x, float point_y,
                                            float origin_x, float origin_y,
                                            float height, float lidar_height) {
    SetMissPointImpl(point_x, point_y, origin_x, origin_y, height, lidar_height,
                     /*unlocked=*/true);
}

void OccupancyGridMap::SetMissPointImpl(float point_x, float point_y,
                                        float origin_x, float origin_y,
                                        float height, float lidar_height,
                                        bool unlocked) {
    if (grids_ == nullptr) {
        return;
    }

    // Diff in absolute cell indices (translation-invariant); origin then mapped.
    const int point_x_index =
        static_cast<int>(std::floor(point_x / options_.resolution));
    const int point_y_index =
        static_cast<int>(std::floor(point_y / options_.resolution));
    int xi_lidar =
        static_cast<int>(std::floor(origin_x / options_.resolution));
    int yi_lidar =
        static_cast<int>(std::floor(origin_y / options_.resolution));

    const int diff_y = point_y_index - yi_lidar;
    const int diff_x = point_x_index - xi_lidar;
    if (diff_y == 0 && diff_x == 0) {
        return;
    }
    if (!WorldToFineIndex(origin_x, origin_y, &xi_lidar, &yi_lidar)) {
        return;
    }

    miss_pts_buf_.clear();
    miss_h_buf_.clear();

    if (std::abs(diff_y) > std::abs(diff_x)) {
        if (diff_y == 0) {
            return;
        }
        const float k = static_cast<float>(diff_x) / static_cast<float>(diff_y);
        const float dh =
            (lidar_height - height) / static_cast<float>(diff_y);
        const int sign = diff_y > 0 ? 1 : -1;
        miss_pts_buf_.reserve(static_cast<std::size_t>(std::abs(diff_y)));
        miss_h_buf_.reserve(static_cast<std::size_t>(std::abs(diff_y)));
        for (int j = sign; j != diff_y; j += sign) {
            const int i = static_cast<int>(static_cast<float>(j) * k);
            miss_pts_buf_.emplace_back(xi_lidar + i, yi_lidar + j);
            miss_h_buf_.emplace_back(lidar_height - static_cast<float>(j) * dh);
        }
    } else {
        if (diff_x == 0) {
            return;
        }
        const float k = static_cast<float>(diff_y) / static_cast<float>(diff_x);
        const float dh =
            (lidar_height - height) / static_cast<float>(diff_x);
        const int sign = diff_x > 0 ? 1 : -1;
        miss_pts_buf_.reserve(static_cast<std::size_t>(std::abs(diff_x)));
        miss_h_buf_.reserve(static_cast<std::size_t>(std::abs(diff_x)));
        for (int i = sign; i != diff_x; i += sign) {
            const int j = static_cast<int>(static_cast<float>(i) * k);
            miss_pts_buf_.emplace_back(xi_lidar + i, yi_lidar + j);
            miss_h_buf_.emplace_back(lidar_height - static_cast<float>(i) * dh);
        }
    }

    for (std::size_t i = 0; i < miss_pts_buf_.size(); ++i) {
        if (unlocked) {
            UpdateCellUnlocked(miss_pts_buf_[i], false, miss_h_buf_[i]);
        } else {
            UpdateCell(miss_pts_buf_[i], false, miss_h_buf_[i]);
        }
    }
}

bool OccupancyGridMap::Occupied(float px, float py) const {
    if (grids_ == nullptr) {
        return false;
    }
    int x_index = 0;
    int y_index = 0;
    if (!WorldToFineIndex(px, py, &x_index, &y_index)) {
        return false;
    }
    const int xi = x_index >> kSubLog2;
    const int yi = y_index >> kSubLog2;
    if (xi < 0 || xi >= grid_size_x_ || yi < 0 || yi >= grid_size_y_) {
        return false;
    }
    if (grids_[xi][yi].empty()) {
        return false;
    }
    const int sxi = x_index - (xi << kSubLog2);
    const int syi = y_index - (yi << kSubLog2);
    unsigned int hit = 0;
    unsigned int visit = 0;
    grids_[xi][yi].GetHitAndVisit(sxi, syi, &hit, &visit);
    if (visit == 0) {
        return false;
    }
    return (static_cast<float>(hit) / static_cast<float>(visit)) >
           options_.occupancy_ratio;
}

cv::Mat OccupancyGridMap::ToCv() const {
    if (grids_ == nullptr || grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        return cv::Mat();
    }
    const int image_width = grid_size_x_ * kSubWidth;
    const int image_height = grid_size_y_ * kSubWidth;
    const cv::Vec3b black(0, 0, 0);
    const cv::Vec3b white(255, 255, 255);
    const cv::Vec3b gray(127, 127, 127);
    cv::Mat image(image_height, image_width, CV_8UC3, gray);

    const int image_height_1 = image_height - 1;
    const int image_width_1 = image_width - 1;

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].empty()) {
                continue;
            }
            for (int sxi = 0; sxi < kSubWidth; ++sxi) {
                for (int syi = 0; syi < kSubWidth; ++syi) {
                    const int x = (bxi << kSubLog2) + sxi;
                    const int y = (byi << kSubLog2) + syi;
                    if (x < 0 || x >= image_width || y < 0 || y >= image_height) {
                        continue;
                    }
                    unsigned int hit_cnt = 0;
                    unsigned int visit_cnt = 0;
                    grids_[bxi][byi].GetHitAndVisit(sxi, syi, &hit_cnt,
                                                    &visit_cnt);
                    const float occ =
                        visit_cnt
                            ? (hit_cnt == 0
                                   ? 0.f
                                   : static_cast<float>(hit_cnt) /
                                         static_cast<float>(visit_cnt))
                            : -1.f;
                    if (occ < 0.f) {
                        continue;
                    }
                    if (occ > options_.occupancy_ratio) {
                        image.at<cv::Vec3b>(y, x) = black;
                    } else {
                        const int y0 = std::max(0, y - 1);
                        const int y1 = std::min(image_height_1, y + 1);
                        const int x0 = std::max(0, x - 1);
                        const int x1 = std::min(image_width_1, x + 1);
                        for (int ey = y0; ey <= y1; ++ey) {
                            for (int ex = x0; ex <= x1; ++ex) {
                                if (image.at<cv::Vec3b>(ey, ex) == gray) {
                                    image.at<cv::Vec3b>(ey, ex) = white;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cv::Mat flipped;
    cv::flip(image, flipped, 1);
    return flipped;
}

void OccupancyGridMap::GetBounds(float* min_x, float* min_y, float* max_x,
                                 float* max_y) const {
    if (min_x != nullptr) {
        *min_x = min_x_;
    }
    if (min_y != nullptr) {
        *min_y = min_y_;
    }
    if (max_x != nullptr) {
        *max_x = max_x_;
    }
    if (max_y != nullptr) {
        *max_y = max_y_;
    }
}

}  // namespace map
}  // namespace argus
