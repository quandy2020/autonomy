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
 * @class EmptyServiceCallPanel
 * @brief Publishes @c std_msgs/Empty (or @c String @c "trigger") on a channel.
 */
class EmptyServiceCallPanel : public QWidget {
  Q_OBJECT

 public:
  explicit EmptyServiceCallPanel(common::VisualizationManager* manager,
                                 QWidget* parent = nullptr);

 private slots:
  void onTriggerClicked();

 private:
  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QPushButton* trigger_button_ = nullptr;
};

}  // namespace autoviz
