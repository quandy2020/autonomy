/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/fathom_node_runner.hpp"

#include "autonomy/perception/fathom/engine/model.hpp"

#include <utility>

namespace autonomy {
namespace perception {
namespace fathom {

FathomNodeRunner::FathomNodeRunner(std::unique_ptr<DepthRefiner> refiner)
    : refiner_(std::move(refiner)) {}

std::unique_ptr<FathomNodeRunner> FathomNodeRunner::Create(
    const FathomConfig& config, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    std::unique_ptr<FathomModelRunner> model = FathomEngine::Create(config, error);
    if (model == nullptr) {
        return nullptr;
    }
    auto refiner = DepthRefiner::Create(config, std::move(model), error);
    if (refiner == nullptr) {
        return nullptr;
    }
    return std::unique_ptr<FathomNodeRunner>(
        new FathomNodeRunner(std::move(refiner)));
}

bool FathomNodeRunner::Process(
    const automsgs::msgs::sensor_msgs::Image& rgb,
    const automsgs::msgs::sensor_msgs::Image& raw_depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
    automsgs::msgs::sensor_msgs::Image* refined_depth,
    automsgs::msgs::sensor_msgs::PointCloud2* point_cloud,
    std::string* error) {
    return refiner_->Refine(rgb, raw_depth, camera_info, refined_depth,
                            point_cloud, error);
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
