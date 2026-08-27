/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QColor>

namespace autoviz {
namespace behavior_tree {

/** Monitor edge colors aligned with Groot `getStyleFromStatus`. */
QColor MonitorEdgeColor(BtNodeStatus status, BtNodeStatus prev_status);

/** Monitor node border emphasis (Groot node_style.NormalBoundaryColor). */
QColor MonitorNodeBorderColor(BtNodeStatus status, BtNodeStatus prev_status);

/** Background chip color for log Previous/Status cells (Groot2 Transitions). */
QColor MonitorStatusChipColor(BtNodeStatus status);

/** Groot port-value highlight (#ffef0b). */
QColor PortValueHighlightColor();

}  // namespace behavior_tree
}  // namespace autoviz
