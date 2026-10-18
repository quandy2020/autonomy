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

#include <cmath>
#include <iostream>

#include "argus/map/occupancy_grid_builder.hpp"
#include "argus/map/occupancy_grid_map.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main() {
    using argus::map::OccupancyGridBuilder;
    using argus::map::OccupancyGridBuilderOptions;
    using argus::map::OccupancyGridMap;
    using argus::map::OccupancyGridMapOptions;
    using argus::sensor::PointCloudXYZI;
    using argus::sensor::PointType;
    using argus::transform::Rigid3d;

    OccupancyGridMapOptions opt;
    opt.resolution = 0.1f;
    opt.occupancy_ratio = 0.3f;
    OccupancyGridMap map(opt);
    if (!map.Init(-2.f, -2.f, 2.f, 2.f)) {
        return Fail("Init");
    }

    map.SetHitPoint(0.5f, 0.0f, true, 0.8f);
    map.SetHitPoint(0.5f, 0.0f, true, 0.8f);
    map.SetHitPoint(0.5f, 0.0f, true, 0.8f);
    map.SetHitPoint(0.5f, 0.0f, true, 0.8f);
    if (!map.Occupied(0.5f, 0.0f)) {
        return Fail("Occupied after hits");
    }

    map.SetMissPoint(0.5f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f);
    const cv::Mat img = map.ToCv();
    if (img.empty() || img.cols < 10 || img.rows < 10) {
        return Fail("ToCv empty");
    }

    if (!map.Resize(-4.f, -4.f, 4.f, 4.f)) {
        return Fail("Resize");
    }
    if (!map.Occupied(0.5f, 0.0f)) {
        return Fail("Occupied after Resize");
    }

    OccupancyGridBuilderOptions bopt;
    bopt.default_floor_height = -1.0;
    bopt.min_th_floor = 0.5;
    bopt.max_th_floor = 1.2;
    bopt.lidar_height = 0.0;
    bopt.map.resolution = 0.2f;
    OccupancyGridBuilder builder(bopt);

    PointCloudXYZI cloud;
    // Corridor walls at y=±1.5, z above floor band (floor=-1 → height≈1.0).
    for (float x = 0.5f; x < 8.f; x += 0.2f) {
        for (float y : {-1.5f, 1.5f}) {
            PointType p;
            p.x = x;
            p.y = y;
            p.z = 0.0f;  // height above floor (-1) = 1.0 ∈ [0.5, 1.2]
            cloud.push_back(p);
        }
    }
    Rigid3d T = Rigid3d::Identity();
    builder.AddScan(T, cloud, false);
    auto g = builder.map();
    if (g == nullptr || g->empty()) {
        return Fail("builder map empty");
    }
    int occ_count = 0;
    for (float x = 1.f; x < 7.f; x += 0.4f) {
        if (g->Occupied(x, 1.5f) || g->Occupied(x, -1.5f)) {
            ++occ_count;
        }
    }
    if (occ_count < 3) {
        return Fail("builder corridor occupancy too sparse");
    }

    std::cout << "OK test_occupancy_grid" << std::endl;
    return 0;
}
