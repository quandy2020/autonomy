/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>

#include <QSplitter>
#include <QTreeWidget>
#include <QWidget>

#include "autoviz/ui/display_tree_delegate.hpp"
#include "autoviz/ui/display_tree_widget.hpp"

class QPushButton;
class QTextBrowser;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}
namespace display {
class Display;
}

class DisplaysPanel : public QWidget {
  Q_OBJECT

 public:
  explicit DisplaysPanel(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  void refresh();
  /** Update channel delegate + display status icons without rebuilding the tree. */
  void refreshStatus();

 signals:
  void fixedFrameChanged(const QString& frame);
  void backgroundColorChanged(const QColor& color);
  void displaysChanged();

 private slots:
  void onDisplayItemChanged(QTreeWidgetItem* item, int column);
  void onAddDisplay();
  void onDuplicateDisplay();
  void onRenameDisplay();
  void onRemoveDisplay();
  void onDisplaySelectionChanged();

 private:
  void setupUi();
  void populateTree();
  void populateGlobalOptions();
  void populateGlobalStatus();
  void populateDisplays();
  void populateDisplayProperties(QTreeWidgetItem* display_item,
                                 display::Display* display,
                                 std::size_t index, int child_index = -1);
  void updateGlobalStatus();
  void updateChannelDelegate();
  void updateHelp(QTreeWidgetItem* item);
  void updateActionButtons();
  std::string uniqueDisplayName(const std::string& base) const;
  QTreeWidgetItem* findItemByKind(DisplayTreeItemKind kind,
                                  int display_index = -1,
                                  const QString& property_key = {}) const;
  void applyPropertyValue(QTreeWidgetItem* item, const QString& value);

  std::shared_ptr<common::VisualizationManager> manager_;
  QSplitter* splitter_ = nullptr;
  DisplayTreeWidget* tree_ = nullptr;
  QTextBrowser* help_ = nullptr;
  DisplayTreeDelegate* value_delegate_ = nullptr;
  QPushButton* duplicate_button_ = nullptr;
  QPushButton* remove_button_ = nullptr;
  QPushButton* rename_button_ = nullptr;
  bool updating_ = false;
  /** Previous error+warn count; used to auto-expand only when issues increase. */
  int global_status_issue_count_ = 0;
};

}  // namespace autoviz
