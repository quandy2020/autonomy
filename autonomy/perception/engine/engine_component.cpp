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
 * @file base_component.cpp
 * @brief YOLO26 base component lifecycle and per-task publication.
 */

#include "autonomy/perception/engine/engine_component.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/proto/perception_options.pb.h"
#include "autonomy/perception/engine/options.hpp"
#include "autonomy/perception/engine/task_pipeline.hpp"

#include <string>
#include <utility>

namespace autonomy {
namespace perception {
namespace base {
namespace {

void Stamp(const automsgs::msgs::sensor_msgs::Image& rgb,
           const std::string& frame_id,
           automsgs::msgs::std_msgs::Header* header) {
    if (header == nullptr) {
        return;
    }
    header->CopyFrom(rgb.header());
    if (!frame_id.empty()) {
        header->set_frame_id(frame_id);
    }
}

}  // namespace

Component::Component() : Component(CreateTaskPipeline) {}

Component::Component(PipelineFactory factory)
    : pipeline_factory_(std::move(factory)) {}

Component::~Component() { Clear(); }

bool Component::Init() {
    ::autonomy::perception::proto::PerceptionOptions config;
    if (!GetProtoConfig(&config) || !config.has_engine()) {
        AERROR << "Missing engine configuration in " << ConfigFilePath();
        return false;
    }
    auto options = config.engine();

    if (options.has_enable_semantic()) {
        bool found = false;
        for (auto& task : *options.mutable_tasks()) {
            if (task.kind() == proto::TASK_SEMANTIC) {
                task.set_enable(options.enable_semantic());
                found = true;
            }
        }
        if (!found) {
            AERROR << "Base semantic override requires a semantic task.";
            return false;
        }
    }

    std::string error;
    if (!ValidateBaseOptions(options, &error)) {
        AERROR << error;
        Clear();
        return false;
    }
    options_ = options;

    if (!pipeline_factory_) {
        AERROR << "Missing perception pipeline factory.";
        return false;
    }
    pipeline_ = pipeline_factory_(options_, &error);
    if (!pipeline_) {
        AERROR << error;
        Clear();
        return false;
    }

    if (TaskEnabled(options_, proto::TASK_SEMANTIC)) {
        semantic_writer_ = node_->CreateWriter<Image>(
            FindTask(options_, proto::TASK_SEMANTIC)->topic());
        if (!semantic_writer_) {
            AERROR << "Base failed to create semantic label writer.";
            Clear();
            return false;
        }
    }

    if (const auto* task = FindTask(options_, proto::TASK_POSE);
        task && task->enable() && !task->keypoints_topic().empty()) {
        keypoints_writer_ = node_->CreateWriter<proto::KeypointFrame>(
            task->keypoints_topic());
        if (!keypoints_writer_) {
            AERROR << "Base failed to create keypoints writer.";
            Clear();
            return false;
        }
    }

    if (const auto* task = FindTask(options_, proto::TASK_SEGMENT);
        task && task->enable() && !task->instance_masks_topic().empty()) {
        instance_masks_writer_ = node_->CreateWriter<proto::InstanceMaskFrame>(
            task->instance_masks_topic());
        if (!instance_masks_writer_) {
            AERROR << "Base failed to create instance mask writer.";
            Clear();
            return false;
        }
    }

    auto bind_det = [&](proto::TaskKind kind, auto* writer, auto* publish,
                        const char* label) {
        if (!TaskEnabled(options_, kind)) {
            return true;
        }
        const auto* task = FindTask(options_, kind);
        *writer = node_->CreateWriter<Detection2DArray>(task->topic());
        if (*writer == nullptr) {
            AERROR << "Base failed to create " << label << " writer.";
            return false;
        }
        *publish = [w = *writer](const Detection2DArray& message) {
            return w->Write(message);
        };
        return true;
    };
    if (!bind_det(proto::TASK_DETECT, &detections_writer_, &publish_detections_,
                  "detections") ||
        !bind_det(proto::TASK_SEGMENT, &masks_writer_, &publish_masks_,
                  "masks") ||
        !bind_det(proto::TASK_POSE, &poses_writer_, &publish_poses_, "poses") ||
        !bind_det(proto::TASK_OBB, &obb_writer_, &publish_obb_, "OBB") ||
        !bind_det(proto::TASK_TRACK, &tracks_writer_, &publish_tracks_,
                  "tracks")) {
        Clear();
        return false;
    }

    if (TaskEnabled(options_, proto::TASK_CLASSIFY)) {
        const auto* task = FindTask(options_, proto::TASK_CLASSIFY);
        classification_writer_ =
            node_->CreateWriter<Classification>(task->topic());
        if (classification_writer_ == nullptr) {
            AERROR << "Base failed to create classification writer.";
            Clear();
            return false;
        }
        publish_classification_ = [this](const Classification& message) {
            return classification_writer_->Write(message);
        };
    }
    if (TaskEnabled(options_, proto::TASK_DEPTH)) {
        const auto* task = FindTask(options_, proto::TASK_DEPTH);
        depth_writer_ = node_->CreateWriter<Image>(task->topic());
        if (depth_writer_ == nullptr) {
            AERROR << "Base failed to create depth writer.";
            Clear();
            return false;
        }
        publish_depth_ = [this](const Image& message) {
            return depth_writer_->Write(message);
        };
    }
    return true;
}

bool Component::ProcessFrame(const Image& rgb, Outputs* outputs,
                             std::string* error) {
    if (outputs == nullptr) {
        if (error != nullptr) {
            *error = "Base: outputs must not be null.";
        }
        return false;
    }

    if (!pipeline_ || !pipeline_->Run(rgb, outputs, error))
        return false;
    if (!outputs->semantic.data().empty())
        Stamp(rgb, options_.camera_frame(), outputs->semantic.mutable_header());

    const std::string& frame = options_.camera_frame();
    if (TaskEnabled(options_, proto::TASK_DETECT)) {
        Stamp(rgb, frame, outputs->detections.mutable_header());
    }
    if (TaskEnabled(options_, proto::TASK_SEGMENT)) {
        Stamp(rgb, frame, outputs->masks.mutable_header());
        Stamp(rgb, frame, outputs->instance_masks.mutable_header());
        for (auto& mask : *outputs->instance_masks.mutable_masks())
            mask.mutable_header()->CopyFrom(outputs->instance_masks.header());
    }
    if (TaskEnabled(options_, proto::TASK_CLASSIFY)) {
        Stamp(rgb, frame, outputs->classification.mutable_header());
    }
    if (TaskEnabled(options_, proto::TASK_POSE)) {
        Stamp(rgb, frame, outputs->poses.mutable_header());
        Stamp(rgb, frame, outputs->keypoints.mutable_header());
    }
    if (TaskEnabled(options_, proto::TASK_OBB)) {
        Stamp(rgb, frame, outputs->obb.mutable_header());
    }
    if (TaskEnabled(options_, proto::TASK_TRACK)) {
        Stamp(rgb, frame, outputs->tracks.mutable_header());
    }
    if (TaskEnabled(options_, proto::TASK_DEPTH)) {
        Stamp(rgb, frame, outputs->depth.mutable_header());
        if (outputs->depth.encoding().empty()) {
            outputs->depth.set_encoding("32FC1");
        }
    }
    return true;
}

bool Component::Proc(const std::shared_ptr<Image>& rgb) {
    if (rgb == nullptr) {
        AERROR << "Base component received a null RGB image.";
        return false;
    }

    Outputs outputs;
    std::string error;
    const bool ok = frame_ ? frame_(*rgb, &outputs, &error)
                           : ProcessFrame(*rgb, &outputs, &error);
    if (!ok) {
        AERROR << "Base frame processing failed: " << error;
        return false;
    }

    if (TaskEnabled(options_, proto::TASK_DETECT) && publish_detections_ &&
        !publish_detections_(outputs.detections)) {
        AERROR << "Base failed to publish detections.";
        return false;
    }
    if (TaskEnabled(options_, proto::TASK_SEGMENT) && publish_masks_ &&
        !publish_masks_(outputs.masks)) {
        AERROR << "Base failed to publish instance masks.";
        return false;
    }
    if (instance_masks_writer_ && !instance_masks_writer_->Write(outputs.instance_masks)) {
        AERROR << "Base failed to publish instance masks.";
        return false;
    }
    if (TaskEnabled(options_, proto::TASK_CLASSIFY) &&
        publish_classification_ &&
        !publish_classification_(outputs.classification)) {
        AERROR << "Base failed to publish classification.";
        return false;
    }
    if (TaskEnabled(options_, proto::TASK_POSE) && publish_poses_ &&
        !publish_poses_(outputs.poses)) {
        AERROR << "Base failed to publish poses.";
        return false;
    }
    if (keypoints_writer_ && !keypoints_writer_->Write(outputs.keypoints)) {
        AERROR << "Base failed to publish pose keypoints.";
        return false;
    }
    if (TaskEnabled(options_, proto::TASK_OBB) && publish_obb_ &&
        !publish_obb_(outputs.obb)) {
        AERROR << "Base failed to publish oriented boxes.";
        return false;
    }
    if (semantic_writer_ && !outputs.semantic.data().empty() &&
        !semantic_writer_->Write(outputs.semantic)) {
        AERROR << "Base failed to publish semantic labels.";
    }
    if (TaskEnabled(options_, proto::TASK_DEPTH) && publish_depth_ &&
        !publish_depth_(outputs.depth)) {
        AERROR << "Base failed to publish depth.";
        return false;
    }
    if (TaskEnabled(options_, proto::TASK_TRACK) && publish_tracks_ &&
        !publish_tracks_(outputs.tracks)) {
        AERROR << "Base failed to publish tracks.";
        return false;
    }
    return true;
}

void Component::Clear() {
    publish_detections_ = nullptr;
    publish_masks_ = nullptr;
    publish_classification_ = nullptr;
    publish_poses_ = nullptr;
    publish_obb_ = nullptr;
    publish_tracks_ = nullptr;
    publish_depth_ = nullptr;
    frame_ = nullptr;
    detections_writer_.reset();
    masks_writer_.reset();
    instance_masks_writer_.reset();
    classification_writer_.reset();
    poses_writer_.reset();
    keypoints_writer_.reset();
    obb_writer_.reset();
    tracks_writer_.reset();
    depth_writer_.reset();
    semantic_writer_.reset();
    pipeline_.reset();
}

}  // namespace base
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::base::Component)
