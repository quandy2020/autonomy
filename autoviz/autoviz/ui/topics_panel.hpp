/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLineEdit;
class QTreeWidget;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/** Foxglove-style Topics tree with drag-to-plot field paths. */
class TopicsPanel : public QWidget {
  Q_OBJECT

 public:
  explicit TopicsPanel(common::VisualizationManager* manager,
                       QWidget* parent = nullptr);

  void refreshChannels();

 signals:
  void seriesDragStarted();

 private:
  void rebuildTree();
  void onItemExpanded(QTreeWidgetItem* item);

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QTreeWidget* tree_ = nullptr;
};

}  // namespace autoviz
