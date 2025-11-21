/**
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

#include "autolink/proto/role_attributes.pb.h"

#include "autolink/transport/common/identity.hpp"

namespace autolink {
namespace transport {

class Endpoint;
using EndpointPtr = std::shared_ptr<Endpoint>;

using proto::RoleAttributes;

class Endpoint
{
public:
    /**
     * @brief Constructor
     * @param attr The role attributes
     */
    explicit Endpoint(const RoleAttributes& attr);

    /**
     * @brief Destructor
     */
    virtual ~Endpoint();

    /**
     * @brief Get the identity
     * @return The identity
     */
    const Identity& id() const {
        return id_;
    }

    /**
     * @brief Get the role attributes
     * @return The role attributes
     */
    const RoleAttributes& attributes() const {
        return attr_;
    }

protected:
    bool enabled_;
    Identity id_;
    RoleAttributes attr_;
};

}  // namespace transport
}  // namespace autolink