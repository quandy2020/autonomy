/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <automsgs/msgs/visualization_msgs/object_fit_command.pb.h>

class QLineEdit;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class ObjectFitOperatorPanel
 * @brief Publishes @c visualization_msgs/ObjectFitCommand (Fit / Clear).
 */
class ObjectFitOperatorPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ObjectFitOperatorPanel(common::VisualizationManager* manager,
                                  QWidget* parent = nullptr);

 private slots:
  void onFitClicked();
  void onClearClicked();

 private:
  void publishCommand(
      automsgs::msgs::visualization_msgs::ObjectFitCommand::Command cmd);

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
};

}  // namespace autoviz
