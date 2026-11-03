/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLineEdit;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class TabletControllerPanel
 * @brief Simple teleop-style pad publishing @c visualization_msgs/StringStamped.
 */
class TabletControllerPanel : public QWidget {
  Q_OBJECT

 public:
  explicit TabletControllerPanel(common::VisualizationManager* manager,
                                 QWidget* parent = nullptr);

 private slots:
  void onDirectionClicked();

 private:
  void publishCommand(const QString& command);

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QPushButton* forward_button_ = nullptr;
  QPushButton* back_button_ = nullptr;
  QPushButton* left_button_ = nullptr;
  QPushButton* right_button_ = nullptr;
  QPushButton* stop_button_ = nullptr;
};

}  // namespace autoviz
