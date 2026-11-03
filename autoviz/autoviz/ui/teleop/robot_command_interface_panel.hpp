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
 * @class RobotCommandInterfacePanel
 * @brief Publishes @c visualization_msgs/StringStamped commands.
 */
class RobotCommandInterfacePanel : public QWidget {
  Q_OBJECT

 public:
  explicit RobotCommandInterfacePanel(common::VisualizationManager* manager,
                                      QWidget* parent = nullptr);

 private slots:
  void onSendClicked();

 private:
  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QLineEdit* command_edit_ = nullptr;
};

}  // namespace autoviz
