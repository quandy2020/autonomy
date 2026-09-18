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

#include "autonomy/localization/atlas/io/g2p5/g2p5_subgrid.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <string>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <opencv2/core.hpp>

namespace autonomy::localization::atlas {
namespace map {

/**
 * Custom 2.5D grid for display / nav export (not used for localization).
 * Stored as SubGrid** with y-major layout; export via ToCV / ToROS.
 */
class G2P5Map {
public:
    struct Options {
        float resolution_ = 0.05f;
        float occupancy_ratio_ = 0.3f;
    };

    explicit G2P5Map(Options options) : options_(options) {
        grid_reso_ = options_.resolution_ * static_cast<float>(sub_grid_width_);
        grids_ = nullptr;
    }

    ~G2P5Map();

    static inline constexpr int SUB_GRID_SIZE = SubGrid::SUB_GRID_SIZE;

    std::shared_ptr<G2P5Map> MakeDeepCopy();

    automsgs::msgs::map_msgs::OccupancyGrid ToROS();
    cv::Mat ToCV();

    //! Write ROS map_server-style `map.pgm` + `map.yaml` under `dir`.
    //! Origin from GetMinAndMax; resolution from GetGridResolution.
    bool SaveOccupancy(const std::string& dir);

    void ReleaseResources();

    bool Init(float temp_min_x, float temp_min_y, float temp_max_x,
              float temp_max_y);
    bool Resize(float temp_min_x, float temp_min_y, float temp_max_x,
                float temp_max_y);

    void SetHitPoint(float px, float py, bool if_hit, float height);
    void SetMissPoint(float point_x, float point_y, float laser_origin_x,
                      float laser_origin_y, float height, float lidar_height);

    bool IsObstacle(const Eigen::Vector2i& point) {
        const int xi = point.x() >> SUB_GRID_SIZE;
        const int yi = point.y() >> SUB_GRID_SIZE;
        if (xi < 0 || xi >= grid_size_x_ || yi < 0 || yi >= grid_size_y_) {
            return false;
        }
        return true;
    }

    void UpdateCell(const Eigen::Vector2i& point_index, bool if_hit,
                    float height);

    bool GetDataIndex(float x, float y, int& x_index, int& y_index);

    bool IsEmpty() { return grids_ == nullptr; }

    void GetMinAndMax(float& min_x, float& min_y, float& max_x,
                      float& max_y) const {
        min_x = min_x_;
        min_y = min_y_;
        max_x = max_x_;
        max_y = max_y_;
    }
    void SetMinAndMax(float min_x, float min_y, float max_x, float max_y) {
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;
    }

    float GetGridResolution() const { return options_.resolution_; }

private:
    static int MapIdx(int sx, int x, int y) { return sx * y + x; }

    float grid_reso_ = 0.0f;
    float min_x_ = 0.f, min_y_ = 0.f, max_x_ = 0.f, max_y_ = 0.f;
    int grid_size_x_ = 0, grid_size_y_ = 0;

    static inline const int sub_grid_width_ = (1 << SUB_GRID_SIZE);

    Options options_;
    SubGrid** grids_ = nullptr;
};

using G2P5MapPtr = std::shared_ptr<G2P5Map>;

}  // namespace map
}  // namespace autonomy::localization::atlas
