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

#include "autonomy/map/strata/point_cloud/point_cloud.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace point_cloud {

PointCloud2DController::PointCloud2DController(render::SceneState::SharedPtr scene,
                                               const std::vector<PointCloudPoint>& points,
                                               PointCloudOptions options)
    : scene_(std::move(scene)), options_(std::move(options)) {
    scene_->pointCloud = points;
    scene_->pointCloudOptions = options_;
    scene_->Touch();
}

void PointCloud2DController::Update(const std::vector<PointCloudPoint>& points) {
    scene_->pointCloud = points;
    scene_->Touch();
}

void PointCloud2DController::Show() {
    options_.use3D = false;
    scene_->pointCloudOptions = options_;
    scene_->Touch();
}

void PointCloud2DController::Hide() {
    scene_->pointCloud.clear();
    scene_->Touch();
}

void PointCloud2DController::Remove() { Hide(); }

std::vector<PointCloudPoint> PointCloud2DController::GetPoints() const {
    return scene_->pointCloud;
}

void PointCloud2DController::UpdatePoint(size_t index, const PointCloudPoint& point) {
    if (index < scene_->pointCloud.size()) {
        scene_->pointCloud[index] = point;
        scene_->Touch();
    }
}

void PointCloud2DController::Set3D(bool enabled) {
    options_.use3D = enabled;
    scene_->pointCloudOptions = options_;
    scene_->Touch();
}

PointCloud3DController::PointCloud3DController(render::SceneState::SharedPtr scene,
                                               const std::vector<PointCloudPoint>& points,
                                               PointCloudOptions options)
    : scene_(std::move(scene)), options_(std::move(options)) {
    options_.use3D = true;
    scene_->pointCloud = points;
    scene_->pointCloudOptions = options_;
    scene_->viewOptions.pitch = 60.;
    scene_->Touch();
}

void PointCloud3DController::Update(const std::vector<PointCloudPoint>& points) {
    scene_->pointCloud = points;
    scene_->Touch();
}

void PointCloud3DController::Show() { scene_->Touch(); }
void PointCloud3DController::Hide() { scene_->pointCloud.clear(); scene_->Touch(); }
void PointCloud3DController::Remove() { Hide(); }
std::vector<PointCloudPoint> PointCloud3DController::GetPoints() const {
    return scene_->pointCloud;
}

}  // namespace point_cloud
}  // namespace strata
}  // namespace map
}  // namespace autonomy
