/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file name_map.hpp
 * @brief Map imported / legacy panel class names to Autoviz dock object names.
 *
 * Used when loading RViz or older Autoviz configs so panel Class/Name fields
 * resolve to current @ref PanelDockWidget @c objectName values.
 *
 * @see FrameSession::loadConfig()
 * @see NormalizePanelObjectName()
 * @see PanelCatalog()
 */

#pragma once

#include <string>

namespace autoviz {

/**
 * @brief Map imported panel Class/Name to Autoviz @ref PanelDockWidget
 *        @c objectName.
 *
 * Accepts RViz-style class ids and historical Autoviz aliases.
 *
 * @param class_or_name Imported class or panel name string.
 * @return Canonical dock object name, or empty / passthrough when unknown
 *         (implementation-defined).
 *
 * @see NormalizePanelObjectName()
 */
std::string MapPanelClassToObjectName(const std::string& class_or_name);

/**
 * @brief Canonical dock @c objectName (legacy aliases → current id).
 *
 * @param object_name Possibly legacy object name from session config.
 * @return Current canonical object name.
 *
 * @see MapPanelClassToObjectName()
 */
std::string NormalizePanelObjectName(const std::string& object_name);

}  // namespace autoviz
