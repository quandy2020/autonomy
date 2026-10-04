/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_discovery.hpp
 * @brief Helpers to list Autolink services and resolve request/response types.
 *
 * Used by Service Call panels to populate service pickers and validate /
 * display protobuf type names for request and response payloads.
 *
 * @see ServiceClientRegistry
 * @see ResolveServiceMessageType
 */

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/**
 * @brief Lists Autolink services currently registered in topology discovery.
 *
 * @return Fully-qualified service names (order depends on Autolink).
 *
 * @note Requires an initialized Autolink / topology context.
 */
std::vector<std::string> ListServices();

/**
 * @brief Resolves the protobuf type name for a service request or response.
 *
 * @param service_name Fully-qualified Autolink service name.
 * @param request When @c true, resolve the request type; otherwise response.
 * @param[out] message_type Filled with the descriptor string on success.
 * @return @c true if the type was resolved; @c false if unknown / unavailable.
 */
bool ResolveServiceMessageType(const std::string& service_name, bool request,
                               std::string* message_type);

}  // namespace integration
}  // namespace autoviz
