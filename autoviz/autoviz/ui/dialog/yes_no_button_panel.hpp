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
 * @class YesNoButtonPanel
 * @brief Publishes @c visualization_msgs/YesNo on a configurable channel.
 */
class YesNoButtonPanel : public QWidget {
  Q_OBJECT

 public:
  explicit YesNoButtonPanel(common::VisualizationManager* manager,
                            QWidget* parent = nullptr);

 private slots:
  void onYesClicked();
  void onNoClicked();

 private:
  void publishYesNo(bool yes);

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QPushButton* yes_button_ = nullptr;
  QPushButton* no_button_ = nullptr;
};

}  // namespace autoviz
