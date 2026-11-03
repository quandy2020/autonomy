/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <automsgs/msgs/visualization_msgs/record_command.pb.h>

class QLineEdit;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class RecordActionPanel
 * @brief Publishes @c visualization_msgs/RecordCommand (Record / Play / Stop).
 */
class RecordActionPanel : public QWidget {
  Q_OBJECT

 public:
  explicit RecordActionPanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

 private slots:
  void onRecordClicked();
  void onPlayClicked();
  void onStopClicked();

 private:
  void publishCommand(automsgs::msgs::visualization_msgs::RecordCommand::Command cmd);

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QLineEdit* file_edit_ = nullptr;
};

}  // namespace autoviz
