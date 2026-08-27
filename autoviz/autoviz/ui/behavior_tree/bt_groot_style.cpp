/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_groot_style.hpp"

namespace autoviz {
namespace behavior_tree {

QColor MonitorEdgeColor(BtNodeStatus status, BtNodeStatus prev_status) {
  if (status == BtNodeStatus::kIdle) {
    if (prev_status == BtNodeStatus::kSuccess) {
      return QColor(100, 150, 100);
    }
    if (prev_status == BtNodeStatus::kRunning) {
      return QColor(80, 160, 110);  // faded green after running
    }
    if (prev_status == BtNodeStatus::kFailure) {
      return QColor(150, 80, 80);
    }
    return QColor(100, 116, 139);
  }
  if (status == BtNodeStatus::kSuccess) {
    return QColor(51, 200, 51);
  }
  if (status == BtNodeStatus::kRunning) {
    // Live signal-flow highlight uses green (not yellow/orange).
    return QColor(0, 200, 83);
  }
  if (status == BtNodeStatus::kFailure) {
    return QColor(250, 50, 50);
  }
  return QColor(100, 116, 139);
}

QColor MonitorNodeBorderColor(BtNodeStatus status, BtNodeStatus prev_status) {
  return MonitorEdgeColor(status, prev_status);
}

QColor MonitorStatusChipColor(BtNodeStatus status) {
  // Groot2 Transitions log cell backgrounds (Previous / Status columns).
  switch (status) {
    case BtNodeStatus::kRunning:
      return QColor(245, 166, 35);   // orange
    case BtNodeStatus::kSuccess:
      return QColor(76, 175, 80);    // green
    case BtNodeStatus::kFailure:
      return QColor(231, 76, 60);    // red
    case BtNodeStatus::kIdle:
    default:
      return QColor(206, 206, 206);  // light gray
  }
}

QColor PortValueHighlightColor() {
  return QColor(0xff, 0xef, 0x0b);
}

}  // namespace behavior_tree
}  // namespace autoviz
