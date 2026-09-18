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

//! mapping/map_incremental — Mapping owns the live IVox; LidarSensor borrows
//! a non-owning pointer via LidarSensor::set_ivox(&MapIncremental::ivox()).
//! Insert path: LidarBridge → IntegrateScan (not LidarSensor::FeedWithPose).
//! Optional TiledMap* mirrors world points for chunked PCD persistence.

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"
#include "autonomy/localization/atlas/mapping/tiled_map.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

class MapIncremental {
public:
    MapIncremental() = default;
    explicit MapIncremental(mapping::IVox::Options opts)
        : ivox_(opts) {}

    void set_tiled_map(TiledMap* tiled) { tiled_map_ = tiled; }
    TiledMap* tiled_map() { return tiled_map_; }
    const TiledMap* tiled_map() const { return tiled_map_; }

    void InsertWorldPoints(const std::vector<Vec3_t>& pts) {
        ivox_.InsertWorldPoints(pts);
        if (tiled_map_) {
            tiled_map_->InsertWorldPoints(pts);
        }
    }

    //! Transform body points to world and insert into IVox (+ optional TiledMap).
    int IntegrateScan(const Mat44_t& T_wb, const std::vector<Vec3_t>& points_body) {
        if (points_body.empty()) {
            return 0;
        }
        const Mat33_t R_wb = T_wb.block<3, 3>(0, 0);
        const Vec3_t t_wb = T_wb.block<3, 1>(0, 3);
        std::vector<Vec3_t> points_world;
        points_world.reserve(points_body.size());
        for (const auto& p : points_body) {
            if (p.allFinite()) {
                points_world.push_back(R_wb * p + t_wb);
            }
        }
        ivox_.InsertWorldPoints(points_world);
        if (tiled_map_) {
            tiled_map_->InsertWorldPoints(points_world);
        }
        return static_cast<int>(points_world.size());
    }

    void Clear() {
        ivox_.Clear();
        if (tiled_map_) {
            tiled_map_->Clear();
        }
    }

    mapping::IVox& ivox() { return ivox_; }
    const mapping::IVox& ivox() const { return ivox_; }

private:
    //! Canonical live local lidar map (owned here, not by LidarSensor).
    mapping::IVox ivox_;
    //! Optional chunked map (owned by Pipeline); non-owning.
    TiledMap* tiled_map_ = nullptr;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
