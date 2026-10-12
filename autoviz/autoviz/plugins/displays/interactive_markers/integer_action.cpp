/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkers IntegerAction
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/integer_action.hpp"

namespace autoviz {
namespace display {

IntegerAction::IntegerAction(const QString& text, QObject* parent, int id)
    : QAction(text, parent), id_(id) {
  connect(this, &QAction::triggered, this, &IntegerAction::emitId);
}

void IntegerAction::emitId() { emit triggered(id_); }

}  // namespace display
}  // namespace autoviz
