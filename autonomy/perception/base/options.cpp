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
 * @file options.cpp
 * @brief Validation for YOLO26 base perception protobuf options.
 */

#include "autonomy/perception/base/options.hpp"

#include <cmath>
#include <string>
#include <unordered_set>

namespace autonomy {
namespace perception {
namespace base {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Base: " + message;
    }
}

bool IsFinite(float value) { return std::isfinite(value); }

const char* TaskKindName(proto::TaskKind kind) {
    switch (kind) {
        case proto::TASK_DETECT:
            return "detect";
        case proto::TASK_SEGMENT:
            return "segment";
        case proto::TASK_CLASSIFY:
            return "classify";
        case proto::TASK_POSE:
            return "pose";
        case proto::TASK_OBB:
            return "obb";
        case proto::TASK_TRACK:
            return "track";
        case proto::TASK_DEPTH:
            return "depth";
        default:
            return "task";
    }
}

bool IsYoloTask(proto::TaskKind kind) {
    return kind == proto::TASK_DETECT || kind == proto::TASK_SEGMENT ||
           kind == proto::TASK_CLASSIFY || kind == proto::TASK_POSE ||
           kind == proto::TASK_OBB || kind == proto::TASK_TRACK;
}

bool ValidTaskEntry(const proto::TaskOptions& task, std::string* error) {
    if (task.kind() == proto::TASK_UNSPECIFIED) {
        SetError(error, "tasks[].kind must be set.");
        return false;
    }
    if (!task.enable()) {
        return true;
    }
    const char* name = TaskKindName(task.kind());
    if (task.model_path().empty()) {
        SetError(error, std::string(name) + " model_path must not be empty.");
        return false;
    }
    if (task.topic().empty() || task.topic().front() != '/') {
        SetError(error, std::string(name) + " topic must be an absolute path.");
        return false;
    }
    if (task.kind() == proto::TASK_POSE && task.num_keypoints() == 0) {
        SetError(error, "num_keypoints must be positive when pose is enabled.");
        return false;
    }
    if (task.kind() == proto::TASK_DEPTH &&
        task.depth_backend() != proto::DEPTH_BACKEND_MOGE) {
        SetError(error, "depth_backend must be DEPTH_BACKEND_MOGE.");
        return false;
    }
    return true;
}

}  // namespace

const proto::TaskOptions* FindTask(const proto::BaseOptions& options,
                                   proto::TaskKind kind) {
    for (const auto& task : options.tasks()) {
        if (task.kind() == kind) {
            return &task;
        }
    }
    return nullptr;
}

bool TaskEnabled(const proto::BaseOptions& options, proto::TaskKind kind) {
    const auto* task = FindTask(options, kind);
    return task != nullptr && task->enable();
}

std::string BackendId(proto::Backend backend) {
    switch (backend) {
        case proto::BACKEND_ONNX:
            return "onnx";
        case proto::BACKEND_TENSORRT:
            return "tensorrt";
        default:
            return {};
    }
}

bool ValidateBaseOptions(const proto::BaseOptions& options, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }

    if (BackendId(options.backend()).empty()) {
        SetError(error, "backend must be BACKEND_ONNX or BACKEND_TENSORRT.");
        return false;
    }
    if (options.input_width() == 0 || options.input_height() == 0) {
        SetError(error, "input_width and input_height must be positive.");
        return false;
    }
    if (!IsFinite(options.confidence_threshold()) ||
        options.confidence_threshold() < 0.0F ||
        options.confidence_threshold() > 1.0F) {
        SetError(error, "confidence_threshold must be in [0, 1].");
        return false;
    }
    if (!IsFinite(options.nms_iou_threshold()) ||
        options.nms_iou_threshold() < 0.0F ||
        options.nms_iou_threshold() > 1.0F) {
        SetError(error, "nms_iou_threshold must be in [0, 1].");
        return false;
    }
    if (options.max_detections() == 0) {
        SetError(error, "max_detections must be positive.");
        return false;
    }
    if (options.camera_frame().empty()) {
        SetError(error, "camera_frame must not be empty.");
        return false;
    }

    std::unordered_set<int> seen;
    bool any = false;
    bool any_yolo = false;
    for (const auto& task : options.tasks()) {
        if (!seen.insert(static_cast<int>(task.kind())).second) {
            SetError(error, "duplicate tasks[].kind entries are not allowed.");
            return false;
        }
        if (!ValidTaskEntry(task, error)) {
            return false;
        }
        if (task.enable()) {
            any = true;
            if (IsYoloTask(task.kind())) {
                any_yolo = true;
            }
        }
    }
    if (!any) {
        SetError(error, "at least one task must be enabled.");
        return false;
    }
    if (any_yolo &&
        (options.input_width() % 32 != 0 || options.input_height() % 32 != 0)) {
        SetError(error, "input size must be divisible by 32.");
        return false;
    }
    if (any_yolo && options.class_names().empty()) {
        SetError(error, "class_names must match the exported class index order.");
        return false;
    }
    return true;
}

}  // namespace base
}  // namespace perception
}  // namespace autonomy
