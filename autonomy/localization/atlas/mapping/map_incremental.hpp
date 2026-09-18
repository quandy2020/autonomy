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

#include "autonomy/localization/atlas/sensor/lidar/lightning/ivox/ivox.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

class MapIncremental {
public:
    MapIncremental() = default;
    explicit MapIncremental(sensor::lightning::IVox::Options opts)
        : ivox_(opts) {}

    void InsertWorldPoints(const std::vector<Vec3_t>& pts) {
        ivox_.InsertWorldPoints(pts);
    }

    void Clear() { ivox_.Clear(); }

    sensor::lightning::IVox& ivox() { return ivox_; }
    const sensor::lightning::IVox& ivox() const { return ivox_; }

private:
    //! Canonical live local lidar map (owned here, not by LidarSensor).
    sensor::lightning::IVox ivox_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
