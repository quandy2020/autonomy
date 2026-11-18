/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file dock_database.hpp
 * @brief Dock plugin and instance database (port of opennav_docking::DockDatabase).
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <automsgs/task/charging.pb.h>
#include "autolink/node/node.hpp"
#include "autonomy/task/charging/dock/types.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::DockDatabase
 * @brief Plugin registry plus named Dock instances (YAML or proto).
 */
class DockDatabase {
public:
    /**
     * @brief Load plugins and dock instances from @p options.
     * @param[in] node Passed to each plugin Configure().
     * @param[in] options DockingOptions.
     * @return true when at least one plugin loaded.
     */
    bool Initialize(const std::shared_ptr<autolink::Node>& node,
                    const ::autonomy::task::proto::DockingOptions& options);

    void Activate();
    void Deactivate();

    bool ReloadFile(const std::string& filepath);

    Dock* FindDock(const std::string& dock_id);
    ChargingDock::Ptr FindDockPlugin(const std::string& type);

    /**
     * @brief Preferred id if present, else first instance.
     * @param[in] preferred Preferred dock name/id.
     * @return Dock map key, or empty when the database is empty.
     */
    std::string SelectDockId(const std::string& preferred) const;

    /**
     * @brief Nearest dock in the XY plane.
     * @param[in] x Map x [m].
     * @param[in] y Map y [m].
     * @return Dock map key, or empty when none.
     */
    std::string NearestDockId(double x, double y) const;

    unsigned int plugin_size() const {
        return static_cast<unsigned int>(plugins_.size());
    }
    unsigned int instance_size() const {
        return static_cast<unsigned int>(instances_.size());
    }

    std::mutex& mutex() { return mutex_; }

private:
    bool LoadPlugins(const std::shared_ptr<autolink::Node>& node,
                     const ::autonomy::task::proto::DockingOptions& options);

    std::shared_ptr<autolink::Node> node_;
    mutable std::mutex mutex_;
    DockPluginMap plugins_;
    DockMap instances_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy
