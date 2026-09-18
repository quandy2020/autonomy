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

#include "autonomy/localization/atlas/io/g2p5/g2p5_map.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace autonomy::localization::atlas {
namespace map {

bool G2P5Map::Init(float temp_min_x, float temp_min_y, float temp_max_x,
                   float temp_max_y) {
    ReleaseResources();
    min_x_ = temp_min_x;
    min_y_ = temp_min_y;
    max_x_ = temp_max_x;
    max_y_ = temp_max_y;

    grid_size_x_ =
        static_cast<int>(std::ceil((max_x_ - min_x_) / grid_reso_));
    grid_size_y_ =
        static_cast<int>(std::ceil((max_y_ - min_y_) / grid_reso_));

    if (grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        return false;
    }

    grids_ = new SubGrid*[static_cast<std::size_t>(grid_size_x_)];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        grids_[xi] = new SubGrid[static_cast<std::size_t>(grid_size_y_)];
    }
    return true;
}

std::shared_ptr<G2P5Map> G2P5Map::MakeDeepCopy() {
    auto ret = std::make_shared<G2P5Map>(options_);
    ret->min_x_ = min_x_;
    ret->min_y_ = min_y_;
    ret->max_x_ = max_x_;
    ret->max_y_ = max_y_;
    ret->grid_size_x_ = grid_size_x_;
    ret->grid_size_y_ = grid_size_y_;
    ret->grid_reso_ = grid_reso_;

    if (grids_ == nullptr || grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        ret->grids_ = nullptr;
        return ret;
    }

    ret->grids_ = new SubGrid*[static_cast<std::size_t>(grid_size_x_)];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        ret->grids_[xi] = new SubGrid[static_cast<std::size_t>(grid_size_y_)];
        for (int yi = 0; yi < grid_size_y_; ++yi) {
            ret->grids_[xi][yi] = grids_[xi][yi];
        }
    }
    return ret;
}

bool G2P5Map::Resize(float temp_min_x, float temp_min_y, float temp_max_x,
                     float temp_max_y) {
    const int temp_grid_size_x =
        static_cast<int>(
            std::ceil((temp_max_x - temp_min_x) / grid_reso_)) +
        1;
    const int temp_grid_size_y =
        static_cast<int>(
            std::ceil((temp_max_y - temp_min_y) / grid_reso_)) +
        1;

    auto** new_grids =
        new SubGrid*[static_cast<std::size_t>(temp_grid_size_x)];
    for (int xi = 0; xi < temp_grid_size_x; ++xi) {
        new_grids[xi] =
            new SubGrid[static_cast<std::size_t>(temp_grid_size_y)];
    }

    const int min_grid_x =
        static_cast<int>(std::round((temp_min_x - min_x_) / grid_reso_));
    const int min_grid_y =
        static_cast<int>(std::round((temp_min_y - min_y_) / grid_reso_));
    const int max_grid_x =
        static_cast<int>(std::ceil((temp_max_x - min_x_) / grid_reso_));
    const int max_grid_y =
        static_cast<int>(std::ceil((temp_max_y - min_y_) / grid_reso_));

    const int dx = min_grid_x < 0 ? 0 : min_grid_x;
    const int dy = min_grid_y < 0 ? 0 : min_grid_y;
    const int Dx = max_grid_x < grid_size_x_ ? max_grid_x : grid_size_x_;
    const int Dy = max_grid_y < grid_size_y_ ? max_grid_y : grid_size_y_;

    if (grids_ != nullptr) {
        for (int x = dx; x < Dx; ++x) {
            for (int y = dy; y < Dy; ++y) {
                assert((x - min_grid_x) >= 0 &&
                       (x - min_grid_x) < temp_grid_size_x);
                assert((y - min_grid_y) >= 0 &&
                       (y - min_grid_y) < temp_grid_size_y);
                new_grids[x - min_grid_x][y - min_grid_y] = grids_[x][y];
            }
        }
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
    }

    grids_ = new_grids;
    min_x_ = temp_min_x;
    min_y_ = temp_min_y;
    max_x_ = temp_max_x;
    max_y_ = temp_max_y;
    grid_size_x_ = temp_grid_size_x;
    grid_size_y_ = temp_grid_size_y;
    return true;
}

G2P5Map::~G2P5Map() { ReleaseResources(); }

void G2P5Map::SetHitPoint(float px, float py, bool if_hit, float height) {
    if (grids_ == nullptr) {
        return;
    }
    if (px < min_x_ || px > max_x_ || py < min_y_ || py > max_y_) {
        return;
    }
    const int x_index =
        static_cast<int>(std::floor((px - min_x_) / options_.resolution_));
    const int y_index =
        static_cast<int>(std::floor((py - min_y_) / options_.resolution_));
    UpdateCell(Eigen::Vector2i(x_index, y_index), if_hit, height);
}

void G2P5Map::UpdateCell(const Eigen::Vector2i& point_index, bool if_hit,
                         float height) {
    if (grids_ == nullptr) {
        return;
    }

    const int x_index = point_index.x();
    const int y_index = point_index.y();
    const int xi = (x_index >> SUB_GRID_SIZE);
    const int yi = (y_index >> SUB_GRID_SIZE);

    if (xi < 0 || xi > (grid_size_x_ - 1) || yi < 0 ||
        yi > (grid_size_y_ - 1)) {
        return;
    }

    const int sub_index_i = x_index - (xi << SUB_GRID_SIZE);
    const int sub_index_j = y_index - (yi << SUB_GRID_SIZE);

    if (sub_index_i < 0 || sub_index_i > (sub_grid_width_ - 1) ||
        sub_index_j < 0 || sub_index_j > (sub_grid_width_ - 1)) {
        return;
    }

    grids_[xi][yi].SetGridHitPoint(if_hit, sub_index_i, sub_index_j, height);
}

void G2P5Map::SetMissPoint(float point_x, float point_y, float laser_origin_x,
                           float laser_origin_y, float height,
                           float lidar_height) {
    if (grids_ == nullptr) {
        return;
    }

    int point_x_index =
        static_cast<int>(std::floor(point_x / options_.resolution_));
    int point_y_index =
        static_cast<int>(std::floor(point_y / options_.resolution_));
    int xi_lidar =
        static_cast<int>(std::floor(laser_origin_x / options_.resolution_));
    int yi_lidar =
        static_cast<int>(std::floor(laser_origin_y / options_.resolution_));

    const int diff_y = point_y_index - yi_lidar;
    const int diff_x = point_x_index - xi_lidar;

    if (diff_y == 0 && diff_x == 0) {
        return;
    }

    if (!GetDataIndex(laser_origin_x, laser_origin_y, xi_lidar, yi_lidar)) {
        return;
    }

    std::vector<Eigen::Vector2i> updated_pts;
    std::vector<float> heights;

    if (std::abs(diff_y) > std::abs(diff_x)) {
        if (diff_y == 0) {
            return;
        }
        const float k = static_cast<float>(diff_x) / static_cast<float>(diff_y);
        const float dh = (lidar_height - height) / static_cast<float>(diff_y);
        const int sign = diff_y > 0 ? 1 : -1;
        for (int j = sign; j != diff_y; j += sign) {
            const int i = static_cast<int>(static_cast<float>(j) * k);
            updated_pts.emplace_back(xi_lidar + i, yi_lidar + j);
            heights.emplace_back(lidar_height -
                                 static_cast<float>(j) * dh);
        }
    } else {
        if (diff_x == 0) {
            return;
        }
        const float k = static_cast<float>(diff_y) / static_cast<float>(diff_x);
        const int sign = diff_x > 0 ? 1 : -1;
        const float dh = (lidar_height - height) / static_cast<float>(diff_x);
        for (int i = sign; i != diff_x; i += sign) {
            const int j = static_cast<int>(static_cast<float>(i) * k);
            updated_pts.emplace_back(xi_lidar + i, yi_lidar + j);
            heights.emplace_back(lidar_height -
                                 static_cast<float>(i) * dh);
        }
    }

    for (std::size_t i = 0; i < updated_pts.size(); ++i) {
        UpdateCell(updated_pts[i], false, heights[i]);
    }
}

bool G2P5Map::GetDataIndex(float x, float y, int& x_index, int& y_index) {
    if (x > max_x_ || x < min_x_ || y > max_y_ || y < min_y_) {
        return false;
    }
    x_index =
        static_cast<int>(std::floor((x - min_x_) / options_.resolution_));
    y_index =
        static_cast<int>(std::floor((y - min_y_) / options_.resolution_));
    return true;
}

void G2P5Map::ReleaseResources() {
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

automsgs::msgs::map_msgs::OccupancyGrid G2P5Map::ToROS() {
    automsgs::msgs::map_msgs::OccupancyGrid occu_map;
    const int image_width = grid_size_x_ * sub_grid_width_;
    const int image_height = grid_size_y_ * sub_grid_width_;

    auto* info = occu_map.mutable_info();
    info->set_resolution(options_.resolution_);
    info->set_width(static_cast<uint32_t>(image_width));
    info->set_height(static_cast<uint32_t>(image_height));
    auto* origin = info->mutable_origin();
    origin->mutable_position()->set_x(min_x_);
    origin->mutable_position()->set_y(min_y_);
    origin->mutable_position()->set_z(0.0);
    origin->mutable_orientation()->set_w(1.0);

    const int grid_map_size = image_width * image_height;
    const int grid_map_size_1 = grid_map_size - 1;
    occu_map.mutable_data()->Resize(grid_map_size, -1);

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].IsEmpty()) {
                continue;
            }
            for (int sxi = 0; sxi < sub_grid_width_; ++sxi) {
                for (int syi = 0; syi < sub_grid_width_; ++syi) {
                    const int x = (bxi << SUB_GRID_SIZE) + sxi;
                    const int y = (byi << SUB_GRID_SIZE) + syi;
                    if (x < 0 || x >= image_width || y < 0 ||
                        y >= image_height) {
                        continue;
                    }
                    unsigned int hit_cnt = 0, visit_cnt = 0;
                    grids_[bxi][byi].GetHitAndVisit(sxi, syi, hit_cnt,
                                                    visit_cnt);
                    const float occ =
                        (visit_cnt > 3)
                            ? static_cast<float>(hit_cnt) /
                                  static_cast<float>(visit_cnt)
                            : -1.f;
                    if (occ < 0.f) {
                        continue;
                    }
                    if (occ > options_.occupancy_ratio_) {
                        (*occu_map.mutable_data())[MapIdx(image_width, x, y)] =
                            100;
                    } else {
                        const int index = MapIdx(image_width, x, y);
                        const int index_min = std::max(0, index - 1);
                        const int index_max =
                            std::min(grid_map_size_1, index + 1);
                        for (int extend = index_min; extend <= index_max;
                             ++extend) {
                            if ((*occu_map.mutable_data())[extend] < 0) {
                                (*occu_map.mutable_data())[extend] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    return occu_map;
}

cv::Mat G2P5Map::ToCV() {
    const int image_width = grid_size_x_ * sub_grid_width_;
    const int image_height = grid_size_y_ * sub_grid_width_;

    const cv::Vec3b black_color(0, 0, 0);
    const cv::Vec3b white_color(255, 255, 255);
    const cv::Vec3b other_color(127, 127, 127);

    cv::Mat image(image_height, image_width, CV_8UC3, other_color);
    const int image_height_1 = image_height - 1;
    const int image_width_1 = image_width - 1;

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].IsEmpty()) {
                continue;
            }
            for (int sxi = 0; sxi < sub_grid_width_; ++sxi) {
                for (int syi = 0; syi < sub_grid_width_; ++syi) {
                    const int x = (bxi << SUB_GRID_SIZE) + sxi;
                    const int y = (byi << SUB_GRID_SIZE) + syi;
                    if (x < 0 || x >= image_width || y < 0 ||
                        y >= image_height) {
                        continue;
                    }
                    unsigned int hit_cnt = 0, visit_cnt = 0;
                    grids_[bxi][byi].GetHitAndVisit(sxi, syi, hit_cnt,
                                                    visit_cnt);
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
                    if (occ > options_.occupancy_ratio_) {
                        image.at<cv::Vec3b>(y, x) = black_color;
                    } else {
                        const int index_y_min = std::max(0, y - 1);
                        const int index_y_max =
                            std::min(image_height_1, y + 1);
                        const int index_x_min = std::max(0, x - 1);
                        const int index_x_max =
                            std::min(image_width_1, x + 1);
                        for (int extend_y = index_y_min;
                             extend_y <= index_y_max; ++extend_y) {
                            for (int extend_x = index_x_min;
                                 extend_x <= index_x_max; ++extend_x) {
                                if (image.at<cv::Vec3b>(extend_y, extend_x) ==
                                    other_color) {
                                    image.at<cv::Vec3b>(extend_y, extend_x) =
                                        white_color;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cv::Mat image_flip;
    cv::flip(image, image_flip, 1);
    return image_flip;
}

}  // namespace map
}  // namespace autonomy::localization::atlas
