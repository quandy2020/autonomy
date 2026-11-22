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

#pragma once
/**
 * @file uploader.hpp
 * @brief Drain P0–P3 queue into a staging/cloud drop directory.
 */
#include <string>
#include "common/base/status.hpp"
#include "robot/data_agent/uploader/upload_queue.hpp"

namespace autoloop {
namespace robot {

class Uploader {
public:
    explicit Uploader(std::string staging);
    void Enqueue(UploadItem item);
    common::Status PumpOnce();
    size_t pending() const { return q_.size(); }

private:
    std::string staging_;
    UploadQueue q_;
};

}  // namespace robot
}  // namespace autoloop
