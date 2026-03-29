/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace aviz {
namespace common {
class Display;
}  // namespace common
}  // namespace aviz

class DisplayContext;

/**
 * @brief Plugin description - metadata about a display plugin
 * Plugin description for the factory
 */
struct PluginDescription {
    std::string class_id;     // e.g., "aviz/RobotModel"
    std::string class_name;   // e.g., "RobotModelDisplay"
    std::string description;  // Human-readable description
    std::string base_class;   // Base class name (always "Display")
    std::vector<std::string> message_types;  // Supported message types (if any)
};

/**
 * @brief Plugin factory for creating Display instances
 * Factory for creating display/tool/controller plugins by class id
 */
class PluginFactory
{
public:
    using DisplayCreator = std::function<std::unique_ptr<aviz::common::Display>(
        const std::string& name)>;

    /**
     * @brief Get singleton instance
     */
    static PluginFactory& Instance();

    /**
     * @brief Register a display plugin
     * @param description Plugin metadata
     * @param creator Factory function to create display instances
     */
    void registerPlugin(const PluginDescription& description,
                        DisplayCreator creator);

    /**
     * @brief Create a display instance by class_id
     * @param class_id Plugin class identifier (e.g., "aviz/RobotModel")
     * @param name Display name
     * @return Display instance or nullptr if not found
     */
    std::unique_ptr<aviz::common::Display> createDisplay(
        const std::string& class_id, const std::string& name);

    /**
     * @brief Get all registered plugin descriptions
     */
    const std::vector<PluginDescription>& getPlugins() const {
        return plugins_;
    }

    /**
     * @brief Get plugin description by class_id
     */
    const PluginDescription* getPluginDescription(
        const std::string& class_id) const;

private:
    PluginFactory() = default;
    ~PluginFactory() = default;
    PluginFactory(const PluginFactory&) = delete;
    PluginFactory& operator=(const PluginFactory&) = delete;

    std::vector<PluginDescription> plugins_;
    std::map<std::string, DisplayCreator> creators_;
    std::map<std::string, size_t> class_id_to_index_;
};
