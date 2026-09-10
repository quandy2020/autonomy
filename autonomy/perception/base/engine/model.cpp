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
 * @file model.cpp
 * @brief Loads one YOLO26 graph and dispatches to the task decoder.
 */

#include "autonomy/perception/base/engine/model.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/perception/base/options.hpp"
#include "autonomy/perception/base/tasks/classify/classify.hpp"
#include "autonomy/perception/base/tasks/depth/depth.hpp"
#include "autonomy/perception/base/tasks/detect/detect.hpp"
#include "autonomy/perception/base/tasks/obb/obb.hpp"
#include "autonomy/perception/base/tasks/pose/pose.hpp"
#include "autonomy/perception/base/tasks/segment/segment.hpp"
#include "autonomy/perception/base/tasks/track/track.hpp"

#include <utility>

namespace autonomy {
namespace perception {
namespace base {
namespace {

std::string ModelPath(Task task, const proto::BaseOptions& options) {
    proto::TaskKind kind = proto::TASK_UNSPECIFIED;
    switch (task) {
        case Task::Detect:
            kind = proto::TASK_DETECT;
            break;
        case Task::Segment:
            kind = proto::TASK_SEGMENT;
            break;
        case Task::Classify:
            kind = proto::TASK_CLASSIFY;
            break;
        case Task::Pose:
            kind = proto::TASK_POSE;
            break;
        case Task::Obb:
            kind = proto::TASK_OBB;
            break;
        case Task::Track:
            kind = proto::TASK_TRACK;
            break;
        case Task::Depth:
            kind = proto::TASK_DEPTH;
            break;
    }
    const auto* entry = FindTask(options, kind);
    return entry != nullptr ? entry->model_path() : std::string{};
}

}  // namespace

Model::Model(Task task, std::string path, proto::BaseOptions options,
             std::unique_ptr<common::network::Engine> engine)
    : task_(task),
      path_(std::move(path)),
      options_(std::move(options)),
      engine_(std::move(engine)) {}

Model::~Model() = default;

std::unique_ptr<Model> Model::Create(Task task, const proto::BaseOptions& options,
                                     std::string* error) {
    const std::string path = ModelPath(task, options);
    const std::string backend = BackendId(options.backend());
    std::string engine_error;
    auto engine =
        common::network::Engine::CreateEngine(path, backend, &engine_error);
    if (engine == nullptr) {
        SetTaskError(error,
                     std::string(TaskName(task)) + " engine: " + engine_error);
        return nullptr;
    }
    return std::unique_ptr<Model>(
        new Model(task, path, options, std::move(engine)));
}

bool Model::Run(const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
                std::string* error) const {
    if (outputs == nullptr) {
        SetTaskError(error, "outputs must not be null.");
        return false;
    }
    if (engine_ == nullptr) {
        SetTaskError(error,
                     std::string(TaskName(task_)) + " engine is not loaded.");
        return false;
    }
    if (rgb.width() == 0 || rgb.height() == 0 || rgb.data().empty()) {
        SetTaskError(error, "RGB image is empty.");
        return false;
    }
    if (rgb.encoding() != "rgb8" && rgb.encoding() != "bgr8") {
        SetTaskError(error, "RGB encoding must be rgb8 or bgr8.");
        return false;
    }

    switch (task_) {
        case Task::Detect:
            return detect::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Segment:
            return segment::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Classify:
            return classify::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Pose:
            return pose::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Obb:
            return obb::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Track:
            return track::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Depth:
            return depth::Decode(*engine_, options_, rgb, outputs, error);
    }
    SetTaskError(error, "unknown task.");
    return false;
}

}  // namespace base
}  // namespace perception
}  // namespace autonomy
