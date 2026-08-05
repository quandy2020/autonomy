/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLineEdit;
class QTreeWidget;
class QTreeWidgetItem;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/** Foxglove-style Channels sidebar: browse Autolink channels and drag fields. */
class ChannelsPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ChannelsPanel(common::VisualizationManager* manager,
                         QWidget* parent = nullptr);

  void refreshChannels();
  void refreshStats();

 private:
  void rebuildTree();
  void applyFilter();
  void onItemExpanded(QTreeWidgetItem* item);
  bool channelsStructureChanged() const;
  void updateChannelStatsColumns();

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  QTimer* stats_timer_ = nullptr;
  QStringList cached_channel_keys_;
};

}  // namespace autoviz
