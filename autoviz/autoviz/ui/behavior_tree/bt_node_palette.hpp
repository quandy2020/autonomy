/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QHash>
#include <QString>
#include <QWidget>

class QLineEdit;
class QTableWidget;
class QToolButton;
class QTreeWidget;

namespace autoviz {
namespace behavior_tree {

/** Draggable palette of registered behavior-tree node types (Groot sidepanel layout). */
class BtNodePalette : public QWidget {
  Q_OBJECT

 public:
  explicit BtNodePalette(QWidget* parent = nullptr);

  void setModels(const QHash<QString, BtNodeModel>& models);
  void setLocked(bool locked);
  bool isLocked() const { return locked_; }

 signals:
  void nodeTypeSelected(const QString& registration_id, BtNodeKind kind);
  void importModelsRequested();
  void exportModelsRequested();
  void addNodeRequested();
  void editNodeRequested(const QString& registration_id);
  void removeNodeRequested(const QString& registration_id);
  void subtreeOpenRequested(const QString& tree_id);
  void lockChanged(bool locked);

 private slots:
  void onAddNodeClicked();
  void onImportClicked();
  void onExportClicked();
  void onLockToggled(bool locked);
  void onContextMenu(const QPoint& pos);

 private:
  void rebuildTree();
  void applyFilter(const QString& text);
  void updatePortsForSelection();
  static QString KindGroupLabel(BtNodeKind kind);

  QToolButton* add_button_ = nullptr;
  QToolButton* import_button_ = nullptr;
  QToolButton* export_button_ = nullptr;
  QToolButton* lock_button_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  QTableWidget* ports_table_ = nullptr;
  QHash<QString, BtNodeModel> models_;
  bool locked_ = false;
};

}  // namespace behavior_tree
}  // namespace autoviz
