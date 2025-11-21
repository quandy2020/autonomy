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

#include <fstream>
#include <sstream>

#include "autonomy/common/configuration_file_resolver.hpp"

namespace autonomy {
namespace transform {

TransformServer::TransformServer()
    : initialized_(false),
      running_(false)
{
    // LOG(INFO) << "TransformServer: Created node 'transform_server'";
    
    // Get TF Buffer singleton
    tf_buffer_ = std::shared_ptr<Buffer>(Buffer::Instance(), [](Buffer*) {});
    
    // Create TF Broadcaster
    tf_broadcaster_ = std::make_shared<TransformBroadcaster>("/tf");
    
    // Create static TF component
    static_transform_ = std::make_shared<StaticTransform>();
}

TransformServer::~TransformServer()
{
    Stop();
}

bool TransformServer::Initialize(const std::string& lua_config_path)
{
    if (initialized_) {
        LOG(WARNING) << "TransformServer already initialized";
        return true;
    }
    
    LOG(INFO) << "Initializing TransformServer from Lua config: " << lua_config_path;
    
    try {
        // Load Lua configuration
        auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(
            std::vector<std::string>{
                ".", 
                "./configuration_files",
                "./configuration_files/transform"
            });
        
        const std::string code = file_resolver->GetFileContentOrDie(lua_config_path);
        auto lua_parameter_dictionary = common::LuaParameterDictionary::NonReferenceCounted(
            code, std::move(file_resolver));
        
        if (!LoadConfigFromLua(lua_parameter_dictionary.get())) {
            LOG(ERROR) << "Failed to load configuration from Lua file";
            return false;
        }
        
        // Initialize TF Buffer
        if (tf_buffer_->Init() != 0) {
            LOG(ERROR) << "Failed to initialize TF Buffer";
            return false;
        }
        
        // Initialize static TF if configuration is provided
        if (!config_.static_transform_config_path.empty()) {
            if (!static_transform_->Initialize(config_.static_transform_config_path)) {
                LOG(WARNING) << "Failed to initialize static transforms";
                // Don't return false, static TF failure doesn't affect the whole system
            }
        } else {
            LOG(INFO) << "No static transform configuration provided";
        }
        
        initialized_ = true;
        LOG(INFO) << "TransformServer initialized successfully";
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception during initialization: " << e.what();
        return false;
    }
}

bool TransformServer::InitializeFromYaml(const std::string& yaml_config_path)
{
    if (initialized_) {
        LOG(WARNING) << "TransformServer already initialized";
        return true;
    }
    
    LOG(INFO) << "Initializing TransformServer from YAML config: " << yaml_config_path;
    
    // Simplified version: directly use YAML path as static TF configuration
    config_.static_transform_config_path = yaml_config_path;
    config_.buffer_cache_time = 10.0;
    config_.default_timeout = 0.01f;
    config_.debug = false;
    config_.publish_rate = 10.0;
    
    return InitializeFromConfig(config_);
}

bool TransformServer::InitializeFromConfig(const TransformServerConfig& config)
{
    if (initialized_) {
        LOG(WARNING) << "TransformServer already initialized";
        return true;
    }
    
    LOG(INFO) << "Initializing TransformServer from config structure";
    
    config_ = config;
    
    try {
        // Initialize TF Buffer
        if (tf_buffer_->Init() != 0) {
            LOG(ERROR) << "Failed to initialize TF Buffer";
            return false;
        }
        
        // Initialize static TF
        if (!config_.static_transform_config_path.empty()) {
            if (!static_transform_->Initialize(config_.static_transform_config_path)) {
                LOG(WARNING) << "Failed to initialize static transforms";
            }
        }
        
        initialized_ = true;
        LOG(INFO) << "TransformServer initialized successfully";
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception during initialization: " << e.what();
        return false;
    }
}

void TransformServer::Start()
{
    if (!initialized_) {
        LOG(ERROR) << "Cannot start: TransformServer not initialized";
        return;
    }
    
    if (running_) {
        LOG(WARNING) << "TransformServer already running";
        return;
    }
    
    LOG(INFO) << "Starting TransformServer";
    
    // Start static TF publishing
    if (static_transform_->GetTransformCount() > 0) {
        static_transform_->Start();
    }
    
    running_ = true;
    LOG(INFO) << "TransformServer started";
}

void TransformServer::Stop()
{
    if (!running_) {
        return;
    }
    
    LOG(INFO) << "Stopping TransformServer";
    
    // Stop static TF publishing
    if (static_transform_) {
        static_transform_->Stop();
    }
    
    running_ = false;
    LOG(INFO) << "TransformServer stopped";
}

commsgs::geometry_msgs::TransformStamped TransformServer::LookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& time,
    const float timeout) const
{
    if (!initialized_) {
        throw std::runtime_error("TransformServer not initialized");
    }
    
    return tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
}

commsgs::geometry_msgs::TransformStamped TransformServer::LookupTransform(
    const std::string& target_frame,
    const commsgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame,
    const float timeout) const
{
    if (!initialized_) {
        throw std::runtime_error("TransformServer not initialized");
    }
    
    return tf_buffer_->lookupTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
}

bool TransformServer::CanTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& time,
    const float timeout,
    std::string* errstr) const
{
    if (!initialized_) {
        if (errstr) {
            *errstr = "TransformServer not initialized";
        }
        return false;
    }
    
    return tf_buffer_->canTransform(target_frame, source_frame, time, timeout, errstr);
}

bool TransformServer::CanTransform(
    const std::string& target_frame,
    const commsgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame,
    const float timeout,
    std::string* errstr) const
{
    if (!initialized_) {
        if (errstr) {
            *errstr = "TransformServer not initialized";
        }
        return false;
    }
    
    return tf_buffer_->canTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame, timeout, errstr);
}

std::vector<std::string> TransformServer::GetAllFrameNames() const
{
    if (!initialized_ || !tf_buffer_) {
        return {};
    }
    
    // Get all frame names from TF Buffer
    std::vector<std::string> frames;
    // TODO: Implement method to get all frames from tf_buffer_
    // This requires tf2::BufferCore to provide the corresponding interface
    
    return frames;
}

void TransformServer::PrintFrameGraph() const
{
    if (!initialized_) {
        LOG(WARNING) << "TransformServer not initialized";
        return;
    }
    
    LOG(INFO) << "=== TF Frame Graph ===";
    
    // Print static transform information
    if (static_transform_) {
        LOG(INFO) << "Static Transforms: " 
                  << static_transform_->GetEnabledTransformCount() << "/" 
                  << static_transform_->GetTransformCount();
    }
    
    // Print all frames
    auto frames = GetAllFrameNames();
    LOG(INFO) << "Total Frames: " << frames.size();
    for (const auto& frame : frames) {
        LOG(INFO) << "  - " << frame;
    }
    
    LOG(INFO) << "=====================";
}

bool TransformServer::LoadConfigFromLua(common::LuaParameterDictionary* dict)
{
    try {
        // Read static TF configuration path
        if (dict->HasKey("static_transform_config")) {
            config_.static_transform_config_path = dict->GetString("static_transform_config");
        }
        
        // Read buffer cache time
        if (dict->HasKey("buffer_cache_time")) {
            config_.buffer_cache_time = dict->GetDouble("buffer_cache_time");
        }
        
        // Read default timeout
        if (dict->HasKey("default_timeout")) {
            config_.default_timeout = static_cast<float>(dict->GetDouble("default_timeout"));
        }
        
        // Read debug flag
        if (dict->HasKey("debug")) {
            config_.debug = dict->GetBool("debug");
        }
        
        // Read publish rate
        if (dict->HasKey("publish_rate")) {
            config_.publish_rate = dict->GetDouble("publish_rate");
        }
        
        LOG(INFO) << "Loaded TF configuration:";
        LOG(INFO) << "  Static TF config: " << config_.static_transform_config_path;
        LOG(INFO) << "  Buffer cache time: " << config_.buffer_cache_time << "s";
        LOG(INFO) << "  Default timeout: " << config_.default_timeout << "s";
        LOG(INFO) << "  Publish rate: " << config_.publish_rate << "Hz";
        LOG(INFO) << "  Debug: " << (config_.debug ? "enabled" : "disabled");
        
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Error loading Lua configuration: " << e.what();
        return false;
    }
}

}  // namespace transform
}  // namespace autonomy
