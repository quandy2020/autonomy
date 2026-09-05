/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/component.hpp"

#include "autonomy/perception/fathom/component_config.hpp"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

bool FathomComponent::Init() {
    proto::FathomComponentConfig component_config;
    if (!GetProtoConfig(&component_config)) {
        AERROR << "Fathom component failed to load config from '"
               << ConfigFilePath() << "'.";
        return false;
    }

    FathomConfig fathom_config;
    FathomComponentTopics topics;
    std::string error;
    if (!TranslateFathomComponentConfig(component_config, &fathom_config,
                                        &topics, &error)) {
        AERROR << error;
        return false;
    }

    runner_ = FathomNodeRunner::Create(fathom_config, &error);
    if (runner_ == nullptr) {
        AERROR << "Fathom component failed to create runner: " << error;
        Clear();
        return false;
    }
    runner_process_ = [this](const Image& rgb, const Image& raw_depth,
                             const CameraInfo& camera_info,
                             Image* refined_depth, PointCloud2* point_cloud,
                             std::string* process_error) {
        return runner_->Process(rgb, raw_depth, camera_info, refined_depth,
                                point_cloud, process_error);
    };

    refined_depth_writer_ = node_->CreateWriter<Image>(topics.refined_depth);
    point_cloud_writer_ = node_->CreateWriter<PointCloud2>(topics.point_cloud);
    if (refined_depth_writer_ == nullptr || point_cloud_writer_ == nullptr) {
        AERROR << "Fathom component failed to create output writers for '"
               << topics.refined_depth << "' and '" << topics.point_cloud
               << "'.";
        Clear();
        return false;
    }
    refined_depth_publish_ = [this](const Image& message) {
        return refined_depth_writer_->Write(message);
    };
    point_cloud_publish_ = [this](const PointCloud2& message) {
        return point_cloud_writer_->Write(message);
    };
    return true;
}

bool FathomComponent::Proc(const std::shared_ptr<Image>& rgb,
                           const std::shared_ptr<Image>& raw_depth,
                           const std::shared_ptr<CameraInfo>& camera_info) {
    if (rgb == nullptr || raw_depth == nullptr || camera_info == nullptr) {
        AERROR << "Fathom component received a null RGB, raw-depth, or "
                  "CameraInfo input.";
        return false;
    }
    if (!runner_process_ || !refined_depth_publish_ || !point_cloud_publish_) {
        AERROR << "Fathom component is not initialized with a runner and both "
                  "output publishers.";
        return false;
    }

    Image refined_depth;
    PointCloud2 point_cloud;
    std::string error;
    if (!runner_process_(*rgb, *raw_depth, *camera_info, &refined_depth,
                         &point_cloud, &error)) {
        AERROR << "Fathom component refinement failed: " << error;
        return false;
    }

    const bool depth_published = refined_depth_publish_(refined_depth);
    const bool cloud_published = point_cloud_publish_(point_cloud);
    if (!depth_published) {
        AERROR << "Fathom component failed to publish refined depth.";
    }
    if (!cloud_published) {
        AERROR << "Fathom component failed to publish point cloud.";
    }
    return depth_published && cloud_published;
}

void FathomComponent::Clear() {
    point_cloud_publish_ = nullptr;
    refined_depth_publish_ = nullptr;
    runner_process_ = nullptr;
    point_cloud_writer_.reset();
    refined_depth_writer_.reset();
    runner_.reset();
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::fathom::FathomComponent)
