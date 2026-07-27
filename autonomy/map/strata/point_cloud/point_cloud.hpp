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

#pragma once

#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace point_cloud {

class PointCloud2DController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(PointCloud2DController)

    PointCloud2DController(render::SceneState::SharedPtr scene,
                             const std::vector<PointCloudPoint>& points,
                             PointCloudOptions options);

    void Update(const std::vector<PointCloudPoint>& points);
    void Show();
    void Hide();
    void Remove();
    std::vector<PointCloudPoint> GetPoints() const;
    void UpdatePoint(size_t index, const PointCloudPoint& point);
    void Set3D(bool enabled);

private:
    render::SceneState::SharedPtr scene_;
    PointCloudOptions options_;
};

class PointCloud3DController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(PointCloud3DController)

    PointCloud3DController(render::SceneState::SharedPtr scene,
                             const std::vector<PointCloudPoint>& points,
                             PointCloudOptions options);

    void Update(const std::vector<PointCloudPoint>& points);
    void Show();
    void Hide();
    void Remove();
    std::vector<PointCloudPoint> GetPoints() const;
    PointCloudOptions GetOptions() const { return options_; }

private:
    render::SceneState::SharedPtr scene_;
    PointCloudOptions options_;
};

}  // namespace point_cloud
}  // namespace strata
}  // namespace map
}  // namespace autonomy
