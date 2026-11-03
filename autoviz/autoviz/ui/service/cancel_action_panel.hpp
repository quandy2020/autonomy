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
 * @class CancelActionPanel
 * @brief Publishes @c std_msgs/String @c "cancel" on a channel.
 */
class CancelActionPanel : public QWidget {
  Q_OBJECT

 public:
  explicit CancelActionPanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

 private slots:
  void onCancelClicked();

 private:
  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QPushButton* cancel_button_ = nullptr;
};

}  // namespace autoviz
