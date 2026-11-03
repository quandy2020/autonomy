/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLineEdit;
class QLabel;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class SelectPointCloudPublishPanel
 * @brief Stub panel — channel field only; selection publish not wired yet.
 */
class SelectPointCloudPublishPanel : public QWidget {
 public:
  explicit SelectPointCloudPublishPanel(
      common::VisualizationManager* manager, QWidget* parent = nullptr);

 private:
  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QLabel* stub_label_ = nullptr;
};

}  // namespace autoviz
