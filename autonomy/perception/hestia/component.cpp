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
 * @file component.cpp
 * @brief Hestia autolink component lifecycle and frame processing.
 */

#include "autonomy/perception/hestia/component.hpp"

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

bool ValidateInputSync(const Component::Image& rgb,
                       const Component::Image& depth,
                       const Component::CameraInfo& camera,
                       const proto::HestiaOptions& options,
                       std::string* error) {
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
    if (rgb.width() != depth.width() || rgb.height() != depth.height()) {
        if (error != nullptr) {
            *error = "RGB and depth dimensions must match.";
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

Component::~Component() { Clear(); }

bool Component::Init() {
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

    if (options_.mode() == proto::MODE_OPEN) {
        open_detector_ = OpenDetector::Create(options_, &error);
        if (open_detector_ == nullptr) {
            AERROR << "Hestia failed to create open-vocabulary detector: "
                   << error;
            Clear();
            return false;
        }
    } else {
        closed_detector_ = ClosedDetector::Create(options_, &error);
        if (closed_detector_ == nullptr) {
            AERROR << "Hestia failed to create closed-set detector: " << error;
            Clear();
            return false;
        }
        if (options_.open_async()) {
            auto async_detector = OpenDetector::Create(options_, &error);
            if (async_detector == nullptr) {
                AERROR << "Hestia failed to create async open-vocabulary "
                          "detector: "
                       << error;
                Clear();
                return false;
            }
            async_open_ =
                std::make_unique<Async<OpenDetector>>(std::move(async_detector));
        } else {
            open_detector_ = OpenDetector::Create(options_, &error);
            if (open_detector_ == nullptr) {
                AERROR << "Hestia failed to create open-vocabulary detector: "
                       << error;
                Clear();
                return false;
            }
        }
        merger_ = std::make_unique<Merger>(options_);
    }

    lifter_ = std::make_unique<Lifter>(options_);
    tracker_ = std::make_unique<Tracker>(options_);
    if (!options_.base_frame().empty()) {
        tf_buffer_ = transform::Buffer::Instance();
    }

    detections_2d_writer_ =
        node_->CreateWriter<Detection2DArray>(options_.detections_2d_topic());
    detections_3d_writer_ =
        node_->CreateWriter<Detection3DArray>(options_.detections_3d_topic());
    if (detections_2d_writer_ == nullptr || detections_3d_writer_ == nullptr) {
        AERROR << "Hestia failed to create detection writers.";
        Clear();
        return false;
    }

    publish_2d_ = [this](const Detection2DArray& message) {
        return detections_2d_writer_->Write(message);
    };
    publish_3d_ = [this](const Detection3DArray& message) {
        return detections_3d_writer_->Write(message);
    };
    return true;
}

bool Component::LookupCameraToBase(
    const Image& rgb,
    automsgs::msgs::geometry_msgs::TransformStamped* transform) const {
    if (transform == nullptr || tf_buffer_ == nullptr ||
        options_.base_frame().empty()) {
        return false;
    }
    try {
        *transform = tf_buffer_->lookupTransform(
            options_.base_frame(), options_.camera_frame(), rgb.header().stamp(),
            options_.tf_timeout_sec());
        return true;
    } catch (const std::exception& ex) {
        AWARN << "Hestia TF lookup " << options_.base_frame() << " <- "
              << options_.camera_frame() << " failed: " << ex.what();
        return false;
    }
}

bool Component::ProcessFrame(const Image& rgb, const Image& depth,
                             const CameraInfo& camera_info,
                             Detection2DArray* detections_2d,
                             Detection3DArray* detections_3d,
                             std::string* error) {
    if (!ValidateInputSync(rgb, depth, camera_info, options_, error)) {
        return false;
    }

    Detection2DArray open_vocab;
    Detection2DArray closed_set;
    Detection2DArray fused;

    if (options_.mode() == proto::MODE_DUAL) {
        if (closed_detector_ == nullptr ||
            !closed_detector_->Detect(rgb, &closed_set, error)) {
            if (error != nullptr && error->empty()) {
                *error = "closed-set detection failed.";
            }
            return false;
        }
        if (async_open_ != nullptr) {
            // Stale-OK: consume the latest completed open result, then enqueue.
            async_open_->TryPoll(&open_vocab);
            async_open_->TryEnqueue(rgb);
        } else if (open_detector_ != nullptr) {
            if (!open_detector_->Detect(rgb, &open_vocab, error)) {
                return false;
            }
        }
        if (merger_ == nullptr) {
            if (error != nullptr) {
                *error = "merger is not initialized.";
            }
            return false;
        }
        merger_->Fuse(closed_set, open_vocab, &fused);
    } else {
        if (open_detector_ == nullptr ||
            !open_detector_->Detect(rgb, &fused, error)) {
            if (error != nullptr && error->empty()) {
                *error = "open-vocabulary detection failed.";
            }
            return false;
        }
    }

    if (tracker_ != nullptr) {
        tracker_->Associate(StampSeconds(rgb.header()), &fused);
    }
    *detections_2d = fused;

    automsgs::msgs::geometry_msgs::TransformStamped camera_to_base;
    const automsgs::msgs::geometry_msgs::TransformStamped* lift_tf = nullptr;
    if (LookupCameraToBase(rgb, &camera_to_base)) {
        lift_tf = &camera_to_base;
    }

    if (lifter_ == nullptr ||
        !lifter_->Lift(fused, depth, camera_info, lift_tf, detections_3d,
                       error)) {
        return false;
    }
    return true;
}

bool Component::Proc(const std::shared_ptr<Image>& rgb,
                     const std::shared_ptr<Image>& depth,
                     const std::shared_ptr<CameraInfo>& camera_info) {
    if (rgb == nullptr || depth == nullptr || camera_info == nullptr) {
        AERROR << "Hestia component received a null RGB, depth, or CameraInfo.";
        return false;
    }
    if (!publish_2d_ || !publish_3d_) {
        AERROR << "Hestia component is not initialized.";
        return false;
    }

    Detection2DArray detections_2d;
    Detection3DArray detections_3d;
    std::string error;
    const bool ok =
        frame_ ? frame_(*rgb, *depth, *camera_info, &detections_2d,
                        &detections_3d, &error)
               : ProcessFrame(*rgb, *depth, *camera_info, &detections_2d,
                              &detections_3d, &error);
    if (!ok) {
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

void Component::Clear() {
    publish_2d_ = nullptr;
    publish_3d_ = nullptr;
    frame_ = nullptr;
    detections_2d_writer_.reset();
    detections_3d_writer_.reset();
    if (async_open_ != nullptr) {
        async_open_->Shutdown();
    }
    async_open_.reset();
    merger_.reset();
    tracker_.reset();
    lifter_.reset();
    closed_detector_.reset();
    open_detector_.reset();
    tf_buffer_ = nullptr;
}

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::hestia::Component)
