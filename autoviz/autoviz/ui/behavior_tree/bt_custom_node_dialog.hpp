/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QDialog>
#include <QHash>
#include <QString>

#include <optional>

class QCloseEvent;

class QComboBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;

namespace autoviz {
namespace behavior_tree {

/** Groot CustomNodeDialog equivalent for TreeNodesModel authoring. */
class BtCustomNodeDialog : public QDialog {
  Q_OBJECT

 public:
  BtCustomNodeDialog(const QHash<QString, BtNodeModel>& models, const QString& edit_id = {},
                     QWidget* parent = nullptr);

  std::optional<BtNodeModel> model() const;
  bool isEditing() const { return editing_; }

 private slots:
  void checkValid();
  void onAddPort();
  void onRemovePort();
  void onTypeChanged(const QString& type);

 protected:
  void closeEvent(QCloseEvent* event) override;

 private:
  void setupUi();
  void loadExistingModel(const BtNodeModel& model);
  void addPortRow(const QString& name, BtPortDirection direction, const QString& default_value,
                  const QString& description, bool editable_key = true,
                  bool editable_direction = true);

  QLineEdit* name_edit_ = nullptr;
  QComboBox* type_combo_ = nullptr;
  QTableWidget* ports_table_ = nullptr;
  QPushButton* add_port_button_ = nullptr;
  QPushButton* remove_port_button_ = nullptr;
  QLabel* warning_label_ = nullptr;
  QDialogButtonBox* button_box_ = nullptr;

  QHash<QString, BtNodeModel> models_;
  bool editing_ = false;
};

}  // namespace behavior_tree
}  // namespace autoviz
