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
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/param_handler.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/transform/transform_broadcaster.hpp"

namespace autonomy {
namespace transform {

/**
 * @brief Static TF transform configuration structure
 */
struct StaticTransformConfig {
    std::string name;
    bool enabled;
    std::string frame_id;
    std::string child_frame_id;
    
    struct Translation {
        double x;
        double y;
        double z;
    } translation;
    
    struct Rotation {
        double x;
        double y;
        double z;
        double w;
    } rotation;
};

/**
 * @brief Global settings
 */
struct StaticTransformSettings {
    double publish_rate = 10.0;
    std::string tf_prefix = "";
    bool print_transforms_on_startup = true;
    bool validate_quaternion = true;
};

/**
 * @brief Static TF transform component
 * 
 * This component loads static TF transforms from a YAML configuration file
 * and periodically publishes these transforms
 */
class StaticTransform
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(StaticTransform)
    
    /**
     * @brief Constructor
     */
     StaticTransform();
    
    /**
     * @brief Destructor
     */
    ~StaticTransform();
    
    /**
     * @brief Initialize the component
     * @param yaml_file_path YAML configuration file path
     * @return true on successful initialization
     */
    bool Initialize(const std::string& yaml_file_path);
    
    /**
     * @brief Start static TF publishing
     */
    void Start();
    
    /**
     * @brief Stop static TF publishing
     */
    void Stop();
    
    /**
     * @brief Get the number of loaded transforms
     */
    size_t GetTransformCount() const { return transforms_.size(); }
    
    /**
     * @brief Get the number of enabled transforms
     */
    size_t GetEnabledTransformCount() const;

private:
    /**
     * @brief Parse configuration from YAML file
     * @param yaml_file_path YAML file path
     * @return true on successful parsing
     */
    bool ParseYamlConfig(const std::string& yaml_file_path);
    
    /**
     * @brief Validate if quaternion is normalized
     * @param rotation Rotation quaternion
     * @return true if valid
     */
    bool ValidateQuaternion(const StaticTransformConfig::Rotation& rotation) const;
    
    /**
     * @brief Publish all static transforms
     */
    void PublishTransforms();
    
    /**
     * @brief Convert configuration to TransformStamped message
     * @param config Configuration
     * @return TransformStamped message
     */
    commsgs::geometry_msgs::TransformStamped ConfigToTransformStamped(
        const StaticTransformConfig& config) const;
    
    /**
     * @brief Print all transform information
     */
    void PrintTransforms() const;

private:
    // Configuration
    common::ParamHandler::SharedPtr param_handler_{nullptr};
    std::vector<StaticTransformConfig> transforms_;
    StaticTransformSettings settings_;
    
    // TF broadcaster
    std::shared_ptr<TransformBroadcaster> tf_broadcaster_;
    
    // Initialization flag
    bool initialized_ = false;
};

}  // namespace transform
}  // namespace autonomy
