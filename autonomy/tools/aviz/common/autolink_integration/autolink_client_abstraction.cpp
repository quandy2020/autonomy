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

#include "autonomy/tools/aviz/common/autolink_integration/autolink_client_abstraction.hpp"

#include <stdexcept>

namespace aviz {
namespace common {
namespace autolink_integration {

AutolinkNodeAbstractionIface::WeakPtr AutolinkClientAbstraction::init(
    int argc, char** argv, const std::string& name, bool anonymous_name) {
    (void)argc;

    if (!autolink::OK()) {
        const char* binary_name = (argv && argv[0]) ? argv[0] : name.c_str();
        // Ignore dag_info for now – aviz just needs basic messaging.
        ::autolink::Init(binary_name, "");
    }

    std::string node_name = name;
    if (anonymous_name) {
        // Simple anonymous scheme: append process id. This keeps node names
        // unique across multiple aviz instances in the same system.
        node_name +=
            "_" + std::to_string(static_cast<unsigned long>(::getpid()));
    }

    // If we already have a node abstraction, make sure we're not silently
    // changing the node name – report an error.
    if (node_abstraction_) {
        if (node_abstraction_->get_node_name() != node_name) {
            throw std::runtime_error(
                "AutolinkClientAbstraction already initialized with node name "
                "'" +
                node_abstraction_->get_node_name() + "', requested '" +
                node_name + "'");
        }
        return node_abstraction_;
    }

    node_abstraction_ = std::make_shared<AutolinkNodeAbstraction>(node_name);

    return node_abstraction_;
}

bool AutolinkClientAbstraction::ok() const {
    return ::autolink::OK() && static_cast<bool>(node_abstraction_);
}

void AutolinkClientAbstraction::shutdown() {
    // Request a graceful shutdown if still running.
    if (!::autolink::IsShutdown()) {
        ::autolink::AsyncShutdown();
        ::autolink::WaitForShutdown();
    }

    // Clear global Autolink state and drop our node abstraction.
    ::autolink::Clear();
    node_abstraction_.reset();
}

}  // namespace autolink_integration
}  // namespace common
}  // namespace aviz
