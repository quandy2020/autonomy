/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class PublishTopicPanel
 * @brief Lightweight publish stub; full editing lives in Autoviz Publish panel.
 */
class PublishTopicPanel : public QWidget {
  Q_OBJECT

 public:
  explicit PublishTopicPanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

 private slots:
  void onPublishClicked();

 private:
  common::VisualizationManager* manager_ = nullptr;
  QLabel* note_label_ = nullptr;
  QLineEdit* channel_edit_ = nullptr;
  QPushButton* publish_button_ = nullptr;
};

}  // namespace autoviz
