/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QPointer>
#include <QWidget>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/tf2/signal.hpp"

class QLineEdit;
class QLabel;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}

class TfTreePanel : public QWidget {
  Q_OBJECT

 public:
  TfTreePanel(transform::Buffer* tf_buffer, common::VisualizationManager* manager,
              QWidget* parent = nullptr);
  ~TfTreePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

 public slots:
  void refresh();

 signals:
  void activated();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;

 private slots:
  void onFrameSelectionChanged();
  void onFilterChanged(const QString& text);

 private:
  struct FrameNode {
    transform::TfFrameStats stats;
    QTreeWidgetItem* item = nullptr;
  };

  void setupUi();
  void rebuildTree();
  void updateDetailsForItem(QTreeWidgetItem* item);
  void updateSummaryLabel(int frame_count, int tree_count);
  QString formatTimestampSec(double sec) const;
  QString formatAgeSec(double age_sec) const;
  double currentTimeSec() const;

  transform::Buffer* tf_buffer_ = nullptr;
  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QLabel* summary_label_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  QLabel* detail_title_ = nullptr;
  QLabel* detail_hint_ = nullptr;
  QWidget* detail_body_ = nullptr;
  QLabel* detail_parent_value_ = nullptr;
  QLabel* detail_type_value_ = nullptr;
  QLabel* detail_authority_value_ = nullptr;
  QLabel* detail_last_time_value_ = nullptr;
  QLabel* detail_age_value_ = nullptr;
  QLabel* detail_count_value_ = nullptr;
  QPointer<QToolButton> expand_button_;
  transform::tf2::VoidSignal::Connection transforms_changed_connection_;
  QHash<QString, FrameNode> frame_nodes_;
  QString selected_frame_id_;
};

}  // namespace autoviz
