/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file config_session.hpp
 * @brief Bridge between @ref SessionConfig and the hierarchical @ref Config tree.
 *
 * Native @c .autoviz files (and imported RViz @c .rviz) are read/written as
 * @ref Config trees via YAML helpers; these functions convert to/from the
 * strongly typed @ref SessionConfig used by VisualizationManager.
 *
 * @see SessionConfig
 * @see Config
 * @see YamlConfigReader
 * @see YamlConfigWriter
 */

#pragma once

#include "autoviz/common/config.hpp"
#include "autoviz/common/session_config.hpp"

namespace autoviz {
namespace common {

/**
 * @brief Serializes a @ref SessionConfig into a native @ref Config tree.
 *
 * @param session Source session (displays, views, panels, window layout, …).
 * @param[out] root Destination config root (must be non-null); overwritten.
 */
void SessionConfigToConfig(const SessionConfig& session, Config* root);

/**
 * @brief Parses a @ref Config root (native or RViz) into @ref SessionConfig.
 *
 * @param root Source config tree (from YAML reader).
 * @param[out] session Destination session (must be non-null).
 * @return @c true on success; @c false if required structure is missing.
 */
bool SessionConfigFromConfig(const Config& root, SessionConfig* session);

}  // namespace common
}  // namespace autoviz
