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

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/transform/proto/transform_options.pb.h"
#include "autonomy/transform/static_transform.hpp"

namespace autonomy {
namespace transform {

/**
 * @class TransformServer
 * @brief Manages static TF transforms loaded from configuration.
 */
class TransformServer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TransformServer)

    /**
     * @brief Construct, load static transforms, and mark the server as running.
     * @param options Transform configuration (e.g. extrinsic YAML path).
     */
    explicit TransformServer(
        const autonomy::transform::proto::TransformOptions& options);

    ~TransformServer();

    /** @return True after construction until destruction. */
    bool IsRunning() const { return running_; }

    /** @return Static transforms loaded at construction. */
    const automsgs::msgs::geometry_msgs::TransformStampeds& GetTransformStampedsData()
        const {
        return static_transform_->GetTransformStampeds();
    }

private:
    // Declaration order must match constructor init: options before StaticTransform.
    autonomy::transform::proto::TransformOptions transform_options_;
    std::unique_ptr<StaticTransform> static_transform_;
    bool running_{false};
};

}  // namespace transform
}  // namespace autonomy
