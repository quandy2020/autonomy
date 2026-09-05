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

/**
 * @file hestia_component.cpp
 * @brief Hestia autolink component lifecycle and frame processing.
 */

#include "autonomy/perception/hestia/hestia_component.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/hestia/options.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

double StampSeconds(const automsgs::msgs::std_msgs::Header& header) {
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

bool InputsFresh(const HestiaComponent::Image& rgb,
                 const HestiaComponent::Image& depth,
                 const HestiaComponent::CameraInfo& camera,
                 const proto::HestiaOptions& options, std::string* error) {
    if (rgb.encoding() != "rgb8" && rgb.encoding() != "bgr8") {
        if (error != nullptr) {
            *error = "RGB encoding must be rgb8 or bgr8.";
        }
        return false;
    }
    if (depth.encoding() != "16UC1" && depth.encoding() != "32FC1") {
        if (error != nullptr) {
            *error = "depth encoding must be 16UC1 or 32FC1.";
        }
        return false;
    }
    if (rgb.width() == 0 || rgb.height() == 0 || depth.width() == 0 ||
        depth.height() == 0 || camera.width() == 0 || camera.height() == 0) {
        if (error != nullptr) {
            *error = "RGB, depth, and camera dimensions must be positive.";
        }
        return false;
    }
    if (depth.width() != camera.width() || depth.height() != camera.height()) {
        if (error != nullptr) {
            *error = "depth and camera dimensions must match.";
        }
        return false;
    }

    const double rgb_t = StampSeconds(rgb.header());
    const double depth_t = StampSeconds(depth.header());
    const double camera_t = StampSeconds(camera.header());
    if (rgb_t > 0.0 && depth_t > 0.0) {
        if (std::fabs(rgb_t - depth_t) > options.max_input_skew_sec()) {
            if (error != nullptr) {
                *error = "RGB/depth timestamp skew exceeds max_input_skew_sec.";
            }
            return false;
        }
    }
    if (rgb_t > 0.0 && camera_t > 0.0) {
        if (std::fabs(rgb_t - camera_t) > options.max_input_skew_sec()) {
            if (error != nullptr) {
                *error =
                    "RGB/camera timestamp skew exceeds max_input_skew_sec.";
            }
            return false;
        }
    }
    return true;
}

}  // namespace

HestiaComponent::~HestiaComponent() { Clear(); }

bool HestiaComponent::Init() {
    proto::HestiaOptions options;
    if (!GetProtoConfig(&options)) {
        AERROR << "Hestia component failed to load config from '"
               << ConfigFilePath() << "'.";
        return false;
    }

    std::string error;
    if (!ValidateHestiaOptions(options, &error)) {
        AERROR << error;
        return false;
    }
    options_ = options;

    if (options_.mode() == "open" || options_.open_async_queue() == 0) {
        open_detector_ = OpenDetector::Create(options_, &error);
        if (open_detector_ == nullptr) {
            AERROR << "Hestia failed to create open detector: " << error;
            Clear();
            return false;
        }
    }

    if (options_.mode() == "dual") {
        home_detector_ = HomeDetector::Create(options_, &error);
        if (home_detector_ == nullptr) {
            AERROR << "Hestia failed to create home detector: " << error;
            Clear();
            return false;
        }
        if (options_.open_async_queue() >= 1) {
            auto async_detector = OpenDetector::Create(options_, &error);
            if (async_detector == nullptr) {
                AERROR << "Hestia failed to create async open detector: "
                       << error;
                Clear();
                return false;
            }
            open_worker_ = std::make_unique<OpenAsyncWorker>(
                std::move(async_detector), options_.open_async_queue());
        }
        merger_ = std::make_unique<DetectionMerger>(options_);
    }

    lifter_ = std::make_unique<DepthLifter>(options_);
    tracker_ = std::make_unique<ObjectTracker>(options_);

    detections_2d_writer_ =
        node_->CreateWriter<Detection2DArray>(options_.detections_2d_topic());
    detections_3d_writer_ =
        node_->CreateWriter<Detection3DArray>(options_.detections_3d_topic());
    if (detections_2d_writer_ == nullptr || detections_3d_writer_ == nullptr) {
        AERROR << "Hestia failed to create detection writers.";
        Clear();
        return false;
    }

    frame_ = [this](const Image& rgb, const Image& depth,
                    const CameraInfo& camera_info, Detection2DArray* d2,
                    Detection3DArray* d3, std::string* process_error) {
        return ProcessFrame(rgb, depth, camera_info, d2, d3, process_error);
    };
    publish_2d_ = [this](const Detection2DArray& message) {
        return detections_2d_writer_->Write(message);
    };
    publish_3d_ = [this](const Detection3DArray& message) {
        return detections_3d_writer_->Write(message);
    };
    return true;
}

bool HestiaComponent::ProcessFrame(const Image& rgb, const Image& depth,
                                   const CameraInfo& camera_info,
                                   Detection2DArray* detections_2d,
                                   Detection3DArray* detections_3d,
                                   std::string* error) {
    if (!InputsFresh(rgb, depth, camera_info, options_, error)) {
        return false;
    }

    Detection2DArray open_detections;
    Detection2DArray home_detections;
    Detection2DArray merged;

    if (options_.mode() == "dual") {
        if (home_detector_ == nullptr ||
            !home_detector_->Detect(rgb, &home_detections, error)) {
            if (error != nullptr && error->empty()) {
                *error = "home detection failed.";
            }
            return false;
        }
        if (open_worker_ != nullptr) {
            open_worker_->TrySubmit(rgb);
            open_worker_->TryGetLatest(&open_detections);
        } else if (open_detector_ != nullptr) {
            if (!open_detector_->Detect(rgb, &open_detections, error)) {
                return false;
            }
        }
        if (merger_ == nullptr) {
            if (error != nullptr) {
                *error = "merger is not initialized.";
            }
            return false;
        }
        merger_->Merge(home_detections, open_detections, &merged);
    } else {
        if (open_detector_ == nullptr ||
            !open_detector_->Detect(rgb, &merged, error)) {
            if (error != nullptr && error->empty()) {
                *error = "open detection failed.";
            }
            return false;
        }
    }

    if (tracker_ != nullptr) {
        tracker_->Update(StampSeconds(rgb.header()), &merged);
    }
    *detections_2d = merged;

    if (lifter_ == nullptr ||
        !lifter_->Lift(merged, depth, camera_info, nullptr, detections_3d,
                       error)) {
        return false;
    }
    return true;
}

bool HestiaComponent::Proc(const std::shared_ptr<Image>& rgb,
                           const std::shared_ptr<Image>& depth,
                           const std::shared_ptr<CameraInfo>& camera_info) {
    if (rgb == nullptr || depth == nullptr || camera_info == nullptr) {
        AERROR << "Hestia component received a null RGB, depth, or CameraInfo.";
        return false;
    }
    if (!frame_ || !publish_2d_ || !publish_3d_) {
        AERROR << "Hestia component is not initialized.";
        return false;
    }

    Detection2DArray detections_2d;
    Detection3DArray detections_3d;
    std::string error;
    if (!frame_(*rgb, *depth, *camera_info, &detections_2d, &detections_3d,
                &error)) {
        AERROR << "Hestia frame processing failed: " << error;
        return false;
    }

    const bool ok_2d = publish_2d_(detections_2d);
    const bool ok_3d = publish_3d_(detections_3d);
    if (!ok_2d) {
        AERROR << "Hestia failed to publish Detection2DArray.";
    }
    if (!ok_3d) {
        AERROR << "Hestia failed to publish Detection3DArray.";
    }
    return ok_2d && ok_3d;
}

void HestiaComponent::Clear() {
    publish_2d_ = nullptr;
    publish_3d_ = nullptr;
    frame_ = nullptr;
    detections_2d_writer_.reset();
    detections_3d_writer_.reset();
    if (open_worker_ != nullptr) {
        open_worker_->Shutdown();
    }
    open_worker_.reset();
    merger_.reset();
    tracker_.reset();
    lifter_.reset();
    home_detector_.reset();
    open_detector_.reset();
}

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::hestia::HestiaComponent)
