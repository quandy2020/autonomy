/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkers IntegerAction
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file integer_action.hpp
 * @brief QAction that emits an integer id (RViz menu entry helper).
 *
 * @see InteractiveMarker
 */

#pragma once

#include <QAction>
#include <QString>

namespace autoviz {
namespace display {

/**
 * @class IntegerAction
 * @brief QAction subclass that keeps an ID and re-emits @c triggered(int).
 */
class IntegerAction : public QAction {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an action with @p text and menu entry @p id.
   */
  IntegerAction(const QString& text, QObject* parent, int id);

  /** Menu entry id associated with this action. */
  int id_ = 0;

 signals:
  /** Emitted when the action is triggered; carries @ref id_. */
  void triggered(int id);

 private slots:
  void emitId();
};

}  // namespace display
}  // namespace autoviz
