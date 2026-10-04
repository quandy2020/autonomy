/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_toolbar.hpp
 * @brief Frosted glass tool strip overlaid on the map viewport.
 *
 * Mirrors @ref ViewportFloatingToolbar styling with map edit tools, measure,
 * and recenter. Empty overlay area is click-through.
 */

#pragma once

#include <functional>

#include <QWidget>

#include "autoviz/ui/map/map_types.hpp"

class QResizeEvent;
class QShowEvent;
class QToolButton;

namespace autoviz {
namespace map {

/**
 * @struct MapToolbarCallbacks
 * @brief Closures for map overlay tool buttons.
 */
struct MapToolbarCallbacks {
  std::function<void(MapEditTool)> on_edit_tool;
  std::function<void()> on_measure;
  std::function<void()> on_recenter;
};

/**
 * @class MapToolbar
 * @brief Right-edge glass toolbar for pan / plan / measure / recenter.
 */
class MapToolbar : public QWidget {
  Q_OBJECT

 public:
  explicit MapToolbar(QWidget* parent = nullptr);

  void setCallbacks(MapToolbarCallbacks callbacks);

  void setEditTool(MapEditTool tool);
  void setMeasureChecked(bool checked);
  void setRecenterToolTip(const QString& tip);

 protected:
  void showEvent(QShowEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  QToolButton* MakeToolButton(const QIcon& icon, const QString& tip,
                              bool checkable = false);
  void updateClickThroughMask();
  void syncEditButtons();

  MapToolbarCallbacks callbacks_;
  MapEditTool edit_tool_ = MapEditTool::kPan;

  QToolButton* pan_button_ = nullptr;
  QToolButton* waypoint_button_ = nullptr;
  QToolButton* geofence_button_ = nullptr;
  QToolButton* rally_button_ = nullptr;
  QToolButton* measure_button_ = nullptr;
  QToolButton* recenter_button_ = nullptr;
};

}  // namespace map
}  // namespace autoviz
