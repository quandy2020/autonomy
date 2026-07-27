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

#include <algorithm>
#include <fstream>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/layers/layer_manager.hpp"
#include "autonomy/map/strata/utils/map_utils.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace layers {

LayerManager::LayerManager(render::SceneState::SharedPtr scene) : scene_(std::move(scene)) {}

SlamMapResult LayerManager::LoadSlamMap(const SlamMapOptions& options) {
    SlamMapResult result;
    const auto corners = utils::GetMapCorners(options.startX, options.startY, options.xGridCount,
                                              options.yGridCount, options.resolution);
    result.corners = corners;

    auto toGps = [&](double x, double y) {
        utils::CartesianToGpsParams params;
        params.x = x;
        params.y = y;
        params.originLatitude = options.originLatitude;
        params.originLongitude = options.originLongitude;
        params.scale = options.scale;
        params.bearing = options.bearing;
        params.zoomFactor = options.zoomFactor;
        const auto gps = utils::CartesianToGps(params);
        LngLat ll;
        ll.x = gps.longitude;
        ll.y = gps.latitude;
        return ll;
    };

    result.coordinates = {
        toGps(corners.bottomLeft.x, corners.bottomLeft.y),
        toGps(corners.bottomRight.x, corners.bottomRight.y),
        toGps(corners.topRight.x, corners.topRight.y),
        toGps(corners.topLeft.x, corners.topLeft.y),
    };
    result.cameraBound = result.coordinates;

    if (!options.imagePath.empty()) {
        std::ifstream file(options.imagePath, std::ios::binary | std::ios::ate);
        if (file) {
            const auto size = file.tellg();
            file.seekg(0, std::ios::beg);
            SlamMapData slam_data;
            slam_data.options = options;
            if (size > 0) {
                slam_data.imageBytes.resize(static_cast<size_t>(size));
                file.read(reinterpret_cast<char*>(slam_data.imageBytes.data()), size);
                slam_data.imageLoaded = static_cast<bool>(file);
            }
            scene_->slamMapData = std::move(slam_data);
            AINFO << "Loaded SLAM map image: " << options.imagePath;
        } else {
            scene_->slamMapData = SlamMapData{.options = options};
            AWARN << "SLAM image not found, using fallback grid: " << options.imagePath;
        }
    } else {
        scene_->slamMapData = SlamMapData{.options = options};
    }

    scene_->slamMap = result;
    AddLayerId(LayerIds::kCanvasMap);
    scene_->Touch();
    return result;
}

void LayerManager::EnsureLayerOrder() {
    const auto& order = LayerOrderArray();
    std::sort(layerIds_.begin(), layerIds_.end(), [&](const std::string& a, const std::string& b) {
        const auto posA = std::find(order.begin(), order.end(), a);
        const auto posB = std::find(order.begin(), order.end(), b);
        const size_t idxA = posA == order.end() ? order.size() : static_cast<size_t>(posA - order.begin());
        const size_t idxB = posB == order.end() ? order.size() : static_cast<size_t>(posB - order.begin());
        return idxA < idxB;
    });
    scene_->activeLayerOrder = layerIds_;
    scene_->Touch();
}

double LayerManager::GetLayerOrder(const std::string& layerId) const {
    const auto& order = LayerOrderArray();
    const auto it = std::find(order.begin(), order.end(), layerId);
    if (it == order.end()) {
        return 999.;
    }
    return static_cast<double>(it - order.begin());
}

void LayerManager::AddLayerId(const std::string& layerId) {
    if (std::find(layerIds_.begin(), layerIds_.end(), layerId) == layerIds_.end()) {
        layerIds_.push_back(layerId);
    }
    EnsureLayerOrder();
}

}  // namespace layers
}  // namespace strata
}  // namespace map
}  // namespace autonomy
