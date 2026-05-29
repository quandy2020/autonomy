/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/transform/transform_server.hpp"

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace transform {

TransformServer::TransformServer(
    const autonomy::transform::proto::TransformOptions& options)
    : transform_options_(options) {
}

bool TransformServer::Initialize() {
    if (initialized_) {
        AWARN << "TransformServer already initialized";
        return true;
    }

    // Create Static Transform component
    static_transform_ = std::make_unique<StaticTransform>(transform_options_);

    initialized_ = true;
    return true;
}

void TransformServer::Start() {
    if (!initialized_) {
        AERROR << "TransformServer not initialized, call Initialize() first";
        return;
    }

    if (running_) {
        AWARN << "TransformServer already running";
        return;
    }

    running_ = true;
}

void TransformServer::Stop() {
    if (!running_) {
        return;
    }

    running_ = false;
}

}  // namespace transform
}  // namespace autonomy
