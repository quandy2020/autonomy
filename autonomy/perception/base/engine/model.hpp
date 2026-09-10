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

#ifndef AUTONOMY_PERCEPTION_BASE_ENGINE_MODEL_HPP_
#define AUTONOMY_PERCEPTION_BASE_ENGINE_MODEL_HPP_

#include "autonomy/perception/base/frame.hpp"
#include "autonomy/perception/base/proto/base.pb.h"
#include "autonomy/perception/base/tasks/task.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace common {
namespace network {
class Engine;
}  // namespace network
}  // namespace common

namespace perception {
namespace base {

class Model
{
public:
    static std::unique_ptr<Model> Create(Task task,
                                         const proto::BaseOptions& options,
                                         std::string* error = nullptr);

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    ~Model();

    Task task() const { return task_; }
    const std::string& path() const { return path_; }

    /**
     * @brief Runs the loaded graph and fills the matching output field.
     *
     * Input: RGB uint8, `images` `[1,3,H,W]` in `[0,1]`, letterboxed to
     * `input_width` x `input_height`. Decode lives under `tasks/<name>/`.
     */
    bool Run(const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
             std::string* error = nullptr) const;

private:
    Model(Task task, std::string path, proto::BaseOptions options,
          std::unique_ptr<common::network::Engine> engine);

    Task task_;
    std::string path_;
    proto::BaseOptions options_;
    std::unique_ptr<common::network::Engine> engine_;
};

}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_BASE_ENGINE_MODEL_HPP_
