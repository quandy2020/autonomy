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

#include "autonomy/perception/engine/binding.hpp"

#include "autonomy/perception/engine/task_pipeline.hpp"

#include "autonomy/perception/models/model_registry.hpp"
#include "autonomy/perception/tasks/task_registry.hpp"
#include "autonomy/perception/engine/options.hpp"
#include "autonomy/perception/tasks/classification/classification_task.hpp"
#include "autonomy/perception/tasks/depth/depth_task.hpp"
#include "autonomy/perception/tasks/detection/detection_task.hpp"
#include "autonomy/perception/tasks/obb/obb_task.hpp"
#include "autonomy/perception/tasks/keypoint/keypoint_task.hpp"
#include "autonomy/perception/tasks/instance_seg/instance_seg_task.hpp"
#include "autonomy/perception/behaviors/tracking/association.hpp"

#include "autonomy/perception/tasks/semantic_seg/semantic_seg_task.hpp"

#include <utility>
#include <vector>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace perception {
namespace base {
namespace {

class Model
{
public:
    using Sessions = ModelSessions;

    static std::unique_ptr<Model> Create(Task task,
                                         const proto::BaseOptions& options,
                                         std::string* error = nullptr, Sessions* sessions = nullptr);

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    ~Model();

    bool SharesDetectionWith(const Model& detector) const;
    void Associate(const Outputs& detections, Outputs* outputs);

    Task task() const { return task_; }
    const std::string& path() const { return path_; }

    /**
     * @brief Runs the loaded graph and fills the matching output field.
     *
     * Input: RGB uint8, `images` `[1,3,H,W]` in `[0,1]`, letterboxed to
     * `input_width` x `input_height`. Decode lives under `tasks/<name>/`.
     */
    bool Run(const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
             std::string* error = nullptr);

private:
    Model(Task task, std::string path, proto::BaseOptions options,
          std::shared_ptr<common::network::Engine> engine);

    track::Associator tracker_;
    Task task_;
    std::string path_;
    proto::BaseOptions options_;
    std::shared_ptr<common::network::Engine> engine_;
};

}  // namespace

Model::Model(Task task, std::string path, proto::BaseOptions options,
             std::shared_ptr<common::network::Engine> engine)
    : task_(task),
      path_(std::move(path)),
      options_(std::move(options)),
      engine_(std::move(engine)) {}

Model::~Model() = default;

std::unique_ptr<Model> Model::Create(Task task, const proto::BaseOptions& options,
                                     std::string* error, Sessions* sessions) {
    const auto* entry = FindTask(options, TaskKindFor(task));
    if (!entry)
        return nullptr;
    const auto binding = ResolveBinding(options, *entry);
    if (!binding) {
        SetTaskError(error, "unknown model_id: " + entry->model_id());
        return nullptr;
    }
    auto engine = LoadRegisteredModel(binding->id, binding->path,
                                      BackendId(binding->backend), sessions, error);
    if (!engine)
        return nullptr;

    auto effective = options;
    effective.set_input_width(binding->width);
    effective.set_input_height(binding->height);
    return std::unique_ptr<Model>(
        new Model(task, binding->path, std::move(effective), std::move(engine)));
}

bool Model::SharesDetectionWith(const Model& detector) const {
    return task_ == Task::Track && detector.task_ == Task::Detect &&
           engine_ == detector.engine_ &&
           OutputName(options_, proto::TASK_TRACK, "output0") ==
               OutputName(detector.options_, proto::TASK_DETECT, "output0") &&
           options_.input_width() == detector.options_.input_width() &&
           options_.input_height() == detector.options_.input_height();
}

void Model::Associate(const Outputs& detections, Outputs* outputs) {
    tracker_.Update(detections.detections, &outputs->tracks);
}

bool Model::Run(const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
                std::string* error) {
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
            return track::Decode(*engine_, options_, rgb, &tracker_, outputs, error);
        case Task::Depth:
            return depth::Decode(*engine_, options_, rgb, outputs, error);
        case Task::Semantic:
            return semantic::Decode(*engine_, options_, rgb, outputs, error);
    }
    SetTaskError(error, "unknown task.");
    return false;
}

namespace {

class TaskPipeline final : public Pipeline {
public:
    bool Load(const proto::BaseOptions& options, std::string* error) {
        Model::Sessions sessions;
        // Run detection before association, independent of declaration order.
        for (const auto task : RegisteredTasks()) {
            const auto kind = TaskKindFor(task);
            if (!TaskEnabled(options, kind))
                continue;
            auto model = Model::Create(task, options, error, &sessions);
            if (!model)
                return false;
            models_.push_back(std::move(model));
        }
        return true;
    }

    bool Run(const automsgs::msgs::sensor_msgs::Image& rgb,
             Outputs* outputs, std::string* error) override {
        Model* detector = nullptr;
        for (const auto& model : models_) {
            if (detector && model->SharesDetectionWith(*detector)) {
                model->Associate(*outputs, outputs);
                continue;
            }
            if (model->task() == Task::Semantic) {
                std::string semantic_error;
                if (!model->Run(rgb, outputs, &semantic_error)) {
                    outputs->semantic.Clear();
                    AERROR << "Base semantic inference failed: " << semantic_error;
                }
                continue;
            }
            if (!model->Run(rgb, outputs, error))
                return false;
            if (model->task() == Task::Detect)
                detector = model.get();
        }
        return true;
    }

private:
    std::vector<std::unique_ptr<Model>> models_;
};

}  // namespace

std::unique_ptr<Pipeline> CreateTaskPipeline(
    const proto::BaseOptions& options, std::string* error) {
    auto pipeline = std::make_unique<TaskPipeline>();
    if (!pipeline->Load(options, error))
        return nullptr;
    return pipeline;
}

}  // namespace base
}  // namespace perception
}  // namespace autonomy
