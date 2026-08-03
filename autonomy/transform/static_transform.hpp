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

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autonomy/transform/proto/transform_options.pb.h"

namespace autonomy {
namespace transform {

/**
 * @brief Static TF transform component
 */
class StaticTransform
{
public:
    /**
     * @brief Define StaticTransform::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(StaticTransform)

    /**
     * @brief Constructor
     * @param options The options for the static transform
     */
    StaticTransform(const autonomy::transform::proto::TransformOptions& options);

    /**
     * @brief Destructor
     */
    ~StaticTransform() = default;

    /**
     * @brief Get the transform stampeds
     * @return The transform stampeds
     */
    const automsgs::msgs::geometry_msgs::TransformStampeds& GetTransformStampeds()
        const {
        return transform_stampeds_;
    }

private:
    /**
     * @brief Send the transforms to the writer
     */
    void SendTransforms();

    /**
     * @brief Send the transform to the writer
     * @param msgtf The transform to send
     */
    void SendTransform(
        const std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& msgtf);

    /**
     * @brief Parse static transforms from the yaml file
     * @param file_path The path to the yaml file
     * @param transforms Output vector of transforms
     * @return True if parsed successfully, false otherwise
     */
    bool ParseFromYaml(
        const std::string& file_path,
        std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& transforms);

    // transform_stampeds
    automsgs::msgs::geometry_msgs::TransformStampeds transform_stampeds_;

    // static_transform_options
    autonomy::transform::proto::TransformOptions static_transform_options_;
};

}  // namespace transform
}  // namespace autonomy
