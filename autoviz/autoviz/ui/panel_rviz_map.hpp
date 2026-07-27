/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

namespace autoviz {

/** Map RViz panel Class/Name to Autoviz PanelDockWidget objectName. */
std::string MapRvizPanelToObjectName(const std::string& rviz_class_or_name);

/** Canonical dock objectName (e.g. Drone3DDock → Vehicle3DDock). */
std::string NormalizePanelObjectName(const std::string& object_name);

}  // namespace autoviz
