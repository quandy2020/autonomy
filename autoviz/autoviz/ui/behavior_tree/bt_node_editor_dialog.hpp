/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QDialog>

class QLineEdit;
class QTabWidget;
class QTextEdit;

namespace autoviz {
namespace behavior_tree {

/** Result of the Groot2-style Node Editor dialog. */
struct BtNodeEditorResult {
  QString instance_name;
  QString skip_if;
  QString success_if;
  QString failure_if;
  QString while_script;
  QString on_success;
  QString on_failure;
  QString on_halted;
  QString post_script;
  QString description;
};

/**
 * Groot2 Node Editor: NodeType / Model Name / Instance name +
 * Pre Conditions / Post Conditions / Description tabs.
 */
class BtNodeEditorDialog : public QDialog {
  Q_OBJECT

 public:
  BtNodeEditorDialog(const BtAbsNode& node, bool read_only, QWidget* parent = nullptr);

  BtNodeEditorResult result() const;

 private:
  QLineEdit* MakeScriptEdit(QWidget* parent, const QString& value, bool read_only);

  QLineEdit* instance_edit_ = nullptr;
  QLineEdit* skip_if_edit_ = nullptr;
  QLineEdit* success_if_edit_ = nullptr;
  QLineEdit* failure_if_edit_ = nullptr;
  QLineEdit* while_edit_ = nullptr;
  QLineEdit* on_success_edit_ = nullptr;
  QLineEdit* on_failure_edit_ = nullptr;
  QLineEdit* on_halted_edit_ = nullptr;
  QLineEdit* post_edit_ = nullptr;
  QTextEdit* description_edit_ = nullptr;
};

}  // namespace behavior_tree
}  // namespace autoviz
