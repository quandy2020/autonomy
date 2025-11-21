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

#include "autonomy/transform/static_transform.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

StaticTransform::StaticTransform()
    : initialized_(false)
{
    tf_broadcaster_ = std::make_shared<TransformBroadcaster>();
}

StaticTransform::~StaticTransform()
{
    Stop();
}

bool StaticTransform::Initialize(const std::string& yaml_file_path)
{
    if (initialized_) {
        LOG(WARNING) << "StaticTransform already initialized";
        return true;
    }
    
    LOG(INFO) << "Initializing StaticTransform from: " << yaml_file_path;
    
    // Create ParamHandler
    param_handler_ = std::make_shared<common::ParamHandler>(yaml_file_path);
    if (!param_handler_->FileOpenedSuccessfully()) {
        LOG(ERROR) << "Failed to open YAML config file: " << yaml_file_path;
        return false;
    }
    
    // Parse YAML configuration file
    if (!ParseYamlConfig(yaml_file_path)) {
        LOG(ERROR) << "Failed to parse YAML config file: " << yaml_file_path;
        return false;
    }
    
    // Print configuration information
    if (settings_.print_transforms_on_startup) {
        PrintTransforms();
    }
    
    initialized_ = true;
    LOG(INFO) << "StaticTransform initialized successfully. "
              << "Loaded " << GetTransformCount() << " transforms, "
              << GetEnabledTransformCount() << " enabled.";
    
    return true;
}

bool StaticTransform::ParseYamlConfig(const std::string& yaml_file_path)
{
    try {
        // Access YAML node using ParamHandler
        auto config = param_handler_->GetConfig();
        
        // Parse global settings (with default values)
        settings_.publish_rate = 10.0;
        settings_.tf_prefix = "";
        settings_.print_transforms_on_startup = true;
        settings_.validate_quaternion = true;
        
        if (config["settings"]) {
            const auto& settings = config["settings"];
            if (settings["publish_rate"]) {
                settings_.publish_rate = settings["publish_rate"].as<double>();
            }
            if (settings["tf_prefix"]) {
                settings_.tf_prefix = settings["tf_prefix"].as<std::string>();
            }
            if (settings["print_transforms_on_startup"]) {
                settings_.print_transforms_on_startup = 
                    settings["print_transforms_on_startup"].as<bool>();
            }
            if (settings["validate_quaternion"]) {
                settings_.validate_quaternion = 
                    settings["validate_quaternion"].as<bool>();
            }
        }
        
        // Parse static transform list
        if (!config["static_transforms"]) {
            LOG(WARNING) << "No 'static_transforms' section found in YAML config";
            return false;
        }
        
        const auto& transforms_node = config["static_transforms"];
        if (!transforms_node.IsSequence()) {
            LOG(ERROR) << "'static_transforms' should be a sequence";
            return false;
        }
        
        for (const auto& transform_node : transforms_node) {
            StaticTransformConfig transform_config;
            
            // Parse required fields
            if (!transform_node["name"] || !transform_node["frame_id"] || 
                !transform_node["child_frame_id"]) {
                LOG(WARNING) << "Skipping transform: missing required fields";
                continue;
            }
            
            transform_config.name = transform_node["name"].as<std::string>();
            transform_config.enabled = transform_node["enabled"].as<bool>(true);
            transform_config.frame_id = transform_node["frame_id"].as<std::string>();
            transform_config.child_frame_id = transform_node["child_frame_id"].as<std::string>();
            
            // Add TF prefix
            if (!settings_.tf_prefix.empty()) {
                transform_config.frame_id = settings_.tf_prefix + "/" + transform_config.frame_id;
                transform_config.child_frame_id = settings_.tf_prefix + "/" + transform_config.child_frame_id;
            }
            
            // Parse translation
            if (transform_node["translation"]) {
                const auto& trans = transform_node["translation"];
                transform_config.translation.x = trans["x"].as<double>(0.0);
                transform_config.translation.y = trans["y"].as<double>(0.0);
                transform_config.translation.z = trans["z"].as<double>(0.0);
            } else {
                transform_config.translation = {0.0, 0.0, 0.0};
            }
            
            // Parse rotation (quaternion)
            if (transform_node["rotation"]) {
                const auto& rot = transform_node["rotation"];
                transform_config.rotation.x = rot["x"].as<double>(0.0);
                transform_config.rotation.y = rot["y"].as<double>(0.0);
                transform_config.rotation.z = rot["z"].as<double>(0.0);
                transform_config.rotation.w = rot["w"].as<double>(1.0);
                
                // Validate quaternion
                if (settings_.validate_quaternion) {
                    if (!ValidateQuaternion(transform_config.rotation)) {
                        LOG(WARNING) << "Transform '" << transform_config.name 
                                    << "' has invalid quaternion, normalizing...";
                        
                        // Normalize quaternion
                        double norm = std::sqrt(
                            transform_config.rotation.x * transform_config.rotation.x +
                            transform_config.rotation.y * transform_config.rotation.y +
                            transform_config.rotation.z * transform_config.rotation.z +
                            transform_config.rotation.w * transform_config.rotation.w);
                        
                        if (norm > 1e-6) {
                            transform_config.rotation.x /= norm;
                            transform_config.rotation.y /= norm;
                            transform_config.rotation.z /= norm;
                            transform_config.rotation.w /= norm;
                        } else {
                            LOG(ERROR) << "Transform '" << transform_config.name 
                                      << "' has zero-length quaternion, using identity";
                            transform_config.rotation = {0.0, 0.0, 0.0, 1.0};
                        }
                    }
                }
            } else {
                // Default: no rotation (identity quaternion)
                transform_config.rotation = {0.0, 0.0, 0.0, 1.0};
            }
            
            transforms_.push_back(transform_config);
        }
        
        if (transforms_.empty()) {
            LOG(WARNING) << "No valid transforms loaded from config";
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Error loading config: " << e.what();
        return false;
    }
}

bool StaticTransform::ValidateQuaternion(
    const StaticTransformConfig::Rotation& rotation) const
{
    double norm = std::sqrt(
        rotation.x * rotation.x +
        rotation.y * rotation.y +
        rotation.z * rotation.z +
        rotation.w * rotation.w);
    
    // Check if close to unit quaternion (allow small error)
    return std::abs(norm - 1.0) < 1e-3;
}

void StaticTransform::Start()
{
    if (!initialized_) {
        LOG(ERROR) << "Cannot start: component not initialized";
        return;
    }
    
    LOG(INFO) << "StaticTransform Start (timer disabled)";
    
    // Create timer to periodically publish TF
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / settings_.publish_rate));
    
    //     period,
    //     [this]() { this->PublishTransforms(); },
    //     true  // auto_start
    // );
    
    LOG(INFO) << "Started publishing static transforms at " 
              << settings_.publish_rate << " Hz";
}

void StaticTransform::Stop()
{
    LOG(INFO) << "StaticTransform Stop (timer disabled)";
}

void StaticTransform::PublishTransforms()
{
    std::vector<commsgs::geometry_msgs::TransformStamped> transforms_to_publish;
    
    // Get current time
    auto now = commsgs::builtin_interfaces::Time::Now();
    
    for (const auto& config : transforms_) {
        if (!config.enabled) {
            continue;
        }
        
        auto transform = ConfigToTransformStamped(config);
        
        // Set timestamp
        transform.header.stamp = now;
        
        transforms_to_publish.push_back(transform);
    }
    
    if (!transforms_to_publish.empty()) {
        tf_broadcaster_->SendTransform(transforms_to_publish);
    }
}

commsgs::geometry_msgs::TransformStamped 
StaticTransform::ConfigToTransformStamped(
    const StaticTransformConfig& config) const
{
    commsgs::geometry_msgs::TransformStamped transform;
    
    // Set frame IDs
    transform.header.frame_id = config.frame_id;
    transform.child_frame_id = config.child_frame_id;
    
    // Set translation
    transform.transform.translation.x = config.translation.x;
    transform.transform.translation.y = config.translation.y;
    transform.transform.translation.z = config.translation.z;
    
    // Set rotation
    transform.transform.rotation.x = config.rotation.x;
    transform.transform.rotation.y = config.rotation.y;
    transform.transform.rotation.z = config.rotation.z;
    transform.transform.rotation.w = config.rotation.w;
    
    return transform;
}

size_t StaticTransform::GetEnabledTransformCount() const
{
    size_t count = 0;
    for (const auto& transform : transforms_) {
        if (transform.enabled) {
            ++count;
        }
    }
    return count;
}

void StaticTransform::PrintTransforms() const
{
    LOG(INFO) << "=== Static Transform Configuration ===";
    LOG(INFO) << "Publish Rate: " << settings_.publish_rate << " Hz";
    LOG(INFO) << "TF Prefix: " << (settings_.tf_prefix.empty() ? "(none)" : settings_.tf_prefix);
    LOG(INFO) << "Total Transforms: " << GetTransformCount();
    LOG(INFO) << "Enabled Transforms: " << GetEnabledTransformCount();
    LOG(INFO) << "";
    
    for (size_t i = 0; i < transforms_.size(); ++i) {
        const auto& tf = transforms_[i];
        LOG(INFO) << "[" << i << "] " << tf.name 
                  << " (" << (tf.enabled ? "ENABLED" : "DISABLED") << ")";
        LOG(INFO) << "  Frame: " << tf.frame_id << " -> " << tf.child_frame_id;
        LOG(INFO) << "  Translation: [" 
                  << tf.translation.x << ", " 
                  << tf.translation.y << ", " 
                  << tf.translation.z << "]";
        LOG(INFO) << "  Rotation: [" 
                  << tf.rotation.x << ", " 
                  << tf.rotation.y << ", " 
                  << tf.rotation.z << ", " 
                  << tf.rotation.w << "]";
    }
    
    LOG(INFO) << "======================================";
}

}  // namespace transform
}  // namespace autonomy
