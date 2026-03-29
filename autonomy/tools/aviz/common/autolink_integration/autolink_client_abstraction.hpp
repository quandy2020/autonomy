/******************************************************************************
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
 *****************************************************************************/

#pragma once

#include <unistd.h>

#include <memory>
#include <string>

#include "autolink/init.hpp"
#include "autolink/state.hpp"
#include "autonomy/tools/aviz/common/autolink_integration/autolink_node_abstraction.hpp"

namespace aviz {
namespace common {
namespace autolink_integration {

/**
 * @brief Thin client-side helper around the global Autolink Init/Clear API.
 *
 * Client-side helper for Autolink init and node abstraction:
 *  - initializes Autolink once per process
 *  - creates a single AutolinkNodeAbstraction used by the visualization
 *  - provides ok() and shutdown() helpers
 */
class AutolinkClientAbstraction
{
public:
    AutolinkClientAbstraction() = default;

    /// @brief Initialize Autolink (if needed) and create a node abstraction.
    ///
    /// \param argc (unused, kept for API symmetry)
    /// \param argv command line arguments (argv[0] is used as binary name)
    /// \param name base node name
    /// \param anonymous_name whether to append a random suffix to the node name
    /// \return weak pointer to the created AutolinkNodeAbstraction
    AutolinkNodeAbstractionIface::WeakPtr init(int argc, char** argv,
                                               const std::string& name,
                                               bool anonymous_name);

    /// @brief Check if Autolink is currently initialized and not shutting down.
    bool ok() const;

    /// @brief Request shutdown of Autolink and clear global state.
    void shutdown();

private:
    std::shared_ptr<AutolinkNodeAbstractionIface> node_abstraction_;
};

}  // namespace autolink_integration
}  // namespace common
}  // namespace aviz
