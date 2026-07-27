/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/config.hpp"
#include "autoviz/common/session_config.hpp"

namespace autoviz {
namespace common {

/** Build native `.autoviz` Config tree from SessionConfig. */
void SessionConfigToConfig(const SessionConfig& session, Config* root);

/** Parse native `.autoviz` or RViz `.rviz` Config root into SessionConfig. */
bool SessionConfigFromConfig(const Config& root, SessionConfig* session);

}  // namespace common
}  // namespace autoviz
