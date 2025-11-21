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

#include "autonomy/transform/proto/transform_options.pb.h"

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/static_transform.hpp"
#include "autonomy/transform/transform_broadcaster.hpp"

namespace autonomy {
namespace transform {

/**
 * @brief Transform Server Configuration
 */
struct TransformServerConfig {
    // Static TF configuration file path
    std::string static_transform_config_path;
    
    // TF buffer cache size (seconds)
    double buffer_cache_time = 10.0;
    
    // TF query timeout (seconds)
    float default_timeout = 0.01f;
    
    // Enable debug mode
    bool debug = false;
    
    // TF publishing frequency (Hz)
    double publish_rate = 10.0;
};

/**
 * @brief Transform Server Class
 * 
 * TransformServer is the core management class of the TF system, responsible for:
 * 1. Managing static TF transform publishing
 * 2. Maintaining the TF transform buffer
 * 3. Providing TF query interface
 * 4. Managing dynamic TF broadcasting
 * 
 * Usage example:
 * @code
 *   auto tf_server = std::make_shared<TransformServer>();
 *   
 *   // Initialize from configuration file
 *   if (tf_server->Initialize("config/transform.lua")) {
 *       tf_server->Start();
 *       
 *       // Query TF
 *       auto transform = tf_server->LookupTransform("map", "base_link", time);
 *       
 *       // Stop
 *       tf_server->Stop();
 *   }
 * @endcode
 */
class TransformServer
{
public:
    /**
     * Define TransformServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TransformServer)
    
    /**
     * @brief Default constructor
     */
    TransformServer();
    
    /**
     * @brief Destructor
     */
    ~TransformServer();
    
    /**
     * @brief Initialize from Lua configuration file
     * @param lua_config_path Lua configuration file path
     * @return true on success
     */
    bool Initialize(const std::string& lua_config_path);
    
    /**
     * @brief Initialize from YAML configuration file
     * @param yaml_config_path YAML configuration file path
     * @return true on success
     */
    bool InitializeFromYaml(const std::string& yaml_config_path);
    
    /**
     * @brief Initialize from configuration structure
     * @param config Configuration structure
     * @return true on success
     */
    bool InitializeFromConfig(const TransformServerConfig& config);
    
    /**
     * @brief Start TF service
     * Starts static TF publishing and other periodic tasks
     */
    void Start();
    
    /**
     * @brief Stop TF service
     */
    void Stop();
    
    /**
     * @brief Query transform between two coordinate frames
     * @param target_frame Target coordinate frame
     * @param source_frame Source coordinate frame
     * @param time Timestamp
     * @param timeout Timeout duration (seconds)
     * @return Transform
     * @throws tf2::TransformException if query fails
     */
    commsgs::geometry_msgs::TransformStamped LookupTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const commsgs::builtin_interfaces::Time& time,
        const float timeout = 0.01f) const;
    
    /**
     * @brief Query transform between two coordinate frames (with fixed frame)
     * @param target_frame Target coordinate frame
     * @param target_time Target time
     * @param source_frame Source coordinate frame
     * @param source_time Source time
     * @param fixed_frame Fixed coordinate frame
     * @param timeout Timeout duration (seconds)
     * @return Transform
     * @throws tf2::TransformException if query fails
     */
    commsgs::geometry_msgs::TransformStamped LookupTransform(
        const std::string& target_frame,
        const commsgs::builtin_interfaces::Time& target_time,
        const std::string& source_frame,
        const commsgs::builtin_interfaces::Time& source_time,
        const std::string& fixed_frame,
        const float timeout = 0.01f) const;
    
    /**
     * @brief Check if transform is possible
     * @param target_frame Target coordinate frame
     * @param source_frame Source coordinate frame
     * @param time Timestamp
     * @param timeout Timeout duration (seconds)
     * @param errstr Error message (optional)
     * @return true if transform is possible
     */
    bool CanTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const commsgs::builtin_interfaces::Time& time,
        const float timeout = 0.01f,
        std::string* errstr = nullptr) const;
    
    /**
     * @brief Check if transform is possible (with fixed frame)
     * @param target_frame Target coordinate frame
     * @param target_time Target time
     * @param source_frame Source coordinate frame
     * @param source_time Source time
     * @param fixed_frame Fixed coordinate frame
     * @param timeout Timeout duration (seconds)
     * @param errstr Error message (optional)
     * @return true if transform is possible
     */
    bool CanTransform(
        const std::string& target_frame,
        const commsgs::builtin_interfaces::Time& target_time,
        const std::string& source_frame,
        const commsgs::builtin_interfaces::Time& source_time,
        const std::string& fixed_frame,
        const float timeout = 0.01f,
        std::string* errstr = nullptr) const;
    
    /**
     * @brief Get TF Buffer
     * @return Shared pointer to TF Buffer
     */
    std::shared_ptr<Buffer> GetBuffer() { return tf_buffer_; }
    
    /**
     * @brief Get TF Buffer (const version)
     * @return Shared pointer to TF Buffer
     */
    std::shared_ptr<const Buffer> GetBuffer() const { return tf_buffer_; }
    
    /**
     * @brief Get Transform Broadcaster
     * @return Shared pointer to Transform Broadcaster
     */
    std::shared_ptr<TransformBroadcaster> GetBroadcaster() { return tf_broadcaster_; }
    
    /**
     * @brief Get static transform component
     * @return Shared pointer to static transform component
     */
    std::shared_ptr<StaticTransform> GetStaticTransform() { return static_transform_; }
    
    /**
     * @brief Check if service is initialized
     * @return true if initialized
     */
    bool IsInitialized() const { return initialized_; }
    
    /**
     * @brief Check if service is running
     * @return true if running
     */
    bool IsRunning() const { return running_; }
    
    /**
     * @brief Get all known coordinate frame names
     * @return List of coordinate frame names
     */
    std::vector<std::string> GetAllFrameNames() const;
    
    /**
     * @brief Print TF tree structure
     */
    void PrintFrameGraph() const;

private:
    /**
     * @brief Load configuration from Lua dictionary
     * @param dict Lua parameter dictionary
     * @return true on success
     */
    bool LoadConfigFromLua(common::LuaParameterDictionary* dict);

private:
    // Configuration
    TransformServerConfig config_;
    
    // TF Buffer
    std::shared_ptr<Buffer> tf_buffer_;
    
    // Static TF publisher
    std::shared_ptr<StaticTransform> static_transform_;
    
    // Dynamic TF broadcaster
    std::shared_ptr<TransformBroadcaster> tf_broadcaster_;
    
    // Status flags
    bool initialized_ = false;
    bool running_ = false;
};

}  // namespace transform
}  // namespace autonomy
