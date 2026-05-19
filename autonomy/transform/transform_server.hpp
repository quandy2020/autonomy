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
 * @brief Transform server that manages static TF transforms
 *
 * This server manages:
 * - Static transforms (published once with latched QoS)
 */
class TransformServer
{
public:
    /**
     * @brief Define TransformServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TransformServer)

    /**
     * @brief Constructor
     * @param options The options for the transform server
     */
    TransformServer(const autonomy::transform::proto::TransformOptions& options);

    /**
     * @brief Destructor
     */
    ~TransformServer() = default;

    /**
     * @brief Initialize the transform server
     * @return True if initialization is successful, false otherwise
     */
    bool Initialize();

    /**
     * @brief Start the transform server
     */
    void Start();

    /**
     * @brief Stop the transform server
     */
    void Stop();

    /**
     * @brief Check if the server is running
     * @return True if running, false otherwise
     */
    bool IsRunning() const {
        return running_;
    }

    /**
     * @brief Get the transform stampeds
     * @return The transform stampeds
     */
    const commsgs::geometry_msgs::TransformStampeds& GetTransformStampedsData()
        const {
        return static_transform_->GetTransformStampeds();
    }

private:
    // Static transform component
    std::unique_ptr<StaticTransform> static_transform_;

    // Transform options
    autonomy::transform::proto::TransformOptions transform_options_;

    // Running state
    bool running_{false};

    // Initialized state
    bool initialized_{false};
};

}  // namespace transform
}  // namespace autonomy
