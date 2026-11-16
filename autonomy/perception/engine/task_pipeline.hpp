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
 * @file model.hpp
 * @brief Shared YOLO26 engine loader. One loaded graph per task.
 */

#ifndef AUTONOMY_PERCEPTION_ENGINE_TASK_PIPELINE_HPP_
#define AUTONOMY_PERCEPTION_ENGINE_TASK_PIPELINE_HPP_

#include "autonomy/perception/engine/pipeline.hpp"
#include "autonomy/perception/proto/perception.pb.h"
#include "autonomy/perception/tasks/task_base.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace base {

std::unique_ptr<Pipeline> CreateTaskPipeline(
    const proto::BaseOptions& options, std::string* error);

}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_ENGINE_TASK_PIPELINE_HPP_
