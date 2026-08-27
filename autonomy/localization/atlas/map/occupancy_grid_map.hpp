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

#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/localization/atlas/map/local_grid.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * Assemble LocalGrids into a global 2D OccupancyGrid (RTAB-Map OccupancyGrid).
 * Uses log-odds fusion: free ← empty/ground, occupied ← obstacles.
 */
class OccupancyGridMap {
public:
    struct Options {
        float cell_size = 0.05f;
        float prob_hit = 0.7f;
        float prob_miss = 0.4f;
        float occupancy_thr = 0.5f;
        float clamping_min = 0.12f;
        float clamping_max = 0.97f;
        /** Dilate free / erode obstacles by N cells (RTAB-Map GridGlobal/Eroded). */
        int erode_obstacles = 0;
        int border_cells = 2;
        int max_width = 2000;
        int max_height = 2000;
    };

    explicit OccupancyGridMap(Options options);

    void Clear();
    void Integrate(const LocalGrid& local);

    /** Binary OccupancyGrid (-1 unknown, 0 free, 100 occ). */
    bool ToMessage(const std::string& frame_id, double timestamp_sec,
                   automsgs::msgs::map_msgs::OccupancyGrid* msg) const;

    /** Probabilistic OccupancyGrid (0–100 = round(100*p), -1 unknown). */
    bool ToProbMessage(const std::string& frame_id, double timestamp_sec,
                       automsgs::msgs::map_msgs::OccupancyGrid* msg) const;

    float cell_size() const { return options_.cell_size; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    float LogOdds(float p) const;
    float ClampLogOdds(float l) const;
    float Prob(float log_odds) const;

    void EnsureContains(float x, float y);
    int Index(int ix, int iy) const;
    bool FillHeader(const std::string& frame_id, double timestamp_sec,
                    automsgs::msgs::map_msgs::OccupancyGrid* msg) const;
    bool IsOccupied(int ix, int iy) const;

    Options options_;
    mutable std::mutex mutex_;
    float origin_x_ = 0.f;
    float origin_y_ = 0.f;
    int width_ = 0;
    int height_ = 0;
    std::vector<float> log_odds_;  // size width*height; NaN = unknown
};

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
