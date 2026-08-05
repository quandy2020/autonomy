/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/** Lists Autolink services registered in the topology. */
std::vector<std::string> ListServices();

/** Resolves protobuf type for a service request or response payload. */
bool ResolveServiceMessageType(const std::string& service_name, bool request,
                               std::string* message_type);

}  // namespace integration
}  // namespace autoviz
