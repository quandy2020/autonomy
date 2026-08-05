/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/** Lists Autolink node names that expose parameter list services. */
std::vector<std::string> ListParameterServerNodes();

}  // namespace integration
}  // namespace autoviz
