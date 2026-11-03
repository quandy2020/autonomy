/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Aggregates operator control panels into one Autoviz dock.
 *****************************************************************************/

#pragma once

#include <QWidget>

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class OperatorToolsPanel
 * @brief Tabbed host for all operator control panels (Yes/No, Record, Fit, …).
 */
class OperatorToolsPanel : public QWidget {
  Q_OBJECT

 public:
  explicit OperatorToolsPanel(common::VisualizationManager* manager,
                         QWidget* parent = nullptr);
};

}  // namespace autoviz
