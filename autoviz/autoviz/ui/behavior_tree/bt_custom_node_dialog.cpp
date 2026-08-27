/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_custom_node_dialog.hpp"

#include <QCloseEvent>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QSet>
#include <QSettings>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QValidator>
#include <QVBoxLayout>

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr char kSharedBlackboardPort[] = "__shared_blackboard";

QString DirectionLabel(BtPortDirection direction) {
  switch (direction) {
    case BtPortDirection::kInput:
      return QStringLiteral("Input");
    case BtPortDirection::kOutput:
      return QStringLiteral("Output");
    case BtPortDirection::kInOut:
      return QStringLiteral("In/Out");
  }
  return QStringLiteral("Input");
}

BtPortDirection DirectionFromLabel(const QString& label) {
  if (label == QLatin1String("Output")) {
    return BtPortDirection::kOutput;
  }
  if (label == QLatin1String("In/Out")) {
    return BtPortDirection::kInOut;
  }
  return BtPortDirection::kInput;
}

BtNodeKind KindFromComboIndex(int index) {
  switch (index) {
    case 0:
      return BtNodeKind::kAction;
    case 1:
      return BtNodeKind::kCondition;
    case 2:
      return BtNodeKind::kControl;
    case 3:
      return BtNodeKind::kSubTree;
    case 4:
      return BtNodeKind::kDecorator;
    default:
      return BtNodeKind::kUndefined;
  }
}

int ComboIndexFromKind(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return 0;
    case BtNodeKind::kCondition:
      return 1;
    case BtNodeKind::kControl:
      return 2;
    case BtNodeKind::kSubTree:
      return 3;
    case BtNodeKind::kDecorator:
      return 4;
    default:
      return 0;
  }
}

}  // namespace

BtCustomNodeDialog::BtCustomNodeDialog(const QHash<QString, BtNodeModel>& models,
                                       const QString& edit_id, QWidget* parent)
    : QDialog(parent), models_(models) {
  setWindowTitle(tr("Custom TreeNode Editor"));
  setupUi();

  QSettings settings;
  restoreGeometry(settings.value(QStringLiteral("BtCustomNodeDialog/geometry")).toByteArray());
  ports_table_->horizontalHeader()->restoreState(
      settings.value(QStringLiteral("BtCustomNodeDialog/header")).toByteArray());

  if (!edit_id.isEmpty() && models_.contains(edit_id)) {
    editing_ = true;
    name_edit_->setText(edit_id);
    loadExistingModel(models_.value(edit_id));
  }

  connect(name_edit_, &QLineEdit::textChanged, this, &BtCustomNodeDialog::checkValid);
  connect(ports_table_, &QTableWidget::cellChanged, this, &BtCustomNodeDialog::checkValid);
  connect(ports_table_->selectionModel(), &QItemSelectionModel::selectionChanged, this,
          [this]() {
            remove_port_button_->setEnabled(!ports_table_->selectionModel()->selectedRows().isEmpty());
          });
  connect(add_port_button_, &QPushButton::clicked, this, &BtCustomNodeDialog::onAddPort);
  connect(remove_port_button_, &QPushButton::clicked, this, &BtCustomNodeDialog::onRemovePort);
  connect(type_combo_, &QComboBox::currentTextChanged, this, &BtCustomNodeDialog::onTypeChanged);
  connect(button_box_, &QDialogButtonBox::accepted, this, &BtCustomNodeDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &BtCustomNodeDialog::reject);

  checkValid();
}

void BtCustomNodeDialog::setupUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(8);

  auto* grid = new QGridLayout();
  auto* name_label = new QLabel(tr("Name:"), this);
  name_edit_ = new QLineEdit(this);
  auto* type_label = new QLabel(tr("Type:"), this);
  type_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  type_combo_ = new QComboBox(this);
  type_combo_->addItems({tr("ActionNode"), tr("ConditionNode"), tr("ControlNode"), tr("SubTree"),
                         tr("DecoratorNode")});
  grid->addWidget(name_label, 0, 0);
  grid->addWidget(name_edit_, 0, 1);
  grid->addWidget(type_label, 0, 2);
  grid->addWidget(type_combo_, 0, 3);
  grid->setColumnStretch(1, 1);
  root->addLayout(grid);

  auto* port_buttons = new QHBoxLayout();
  port_buttons->addStretch(1);
  add_port_button_ = new QPushButton(tr("Add port"), this);
  remove_port_button_ = new QPushButton(tr("Remove"), this);
  remove_port_button_->setEnabled(false);
  port_buttons->addWidget(add_port_button_);
  port_buttons->addWidget(remove_port_button_);
  root->addLayout(port_buttons);

  ports_table_ = new QTableWidget(this);
  ports_table_->setColumnCount(4);
  ports_table_->setHorizontalHeaderLabels(
      {tr("Port Name"), tr("Direction"), tr("Default value"), tr("Description")});
  ports_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  ports_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ports_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Interactive);
  ports_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  ports_table_->verticalHeader()->setVisible(false);
  ports_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  ports_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  root->addWidget(ports_table_, 1);

  warning_label_ = new QLabel(tr("Warning…"), this);
  warning_label_->setStyleSheet(QStringLiteral("color: #cc0000;"));
  root->addWidget(warning_label_);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(false);
  root->addWidget(button_box_);
}

void BtCustomNodeDialog::loadExistingModel(const BtNodeModel& model) {
  type_combo_->setCurrentIndex(ComboIndexFromKind(model.kind));
  if (model.kind == BtNodeKind::kSubTree) {
    type_combo_->setEnabled(false);
  }
  for (const BtPortModel& port : model.ports) {
    addPortRow(port.name, port.direction, port.default_value, port.description);
  }
}

void BtCustomNodeDialog::addPortRow(const QString& name, BtPortDirection direction,
                                    const QString& default_value, const QString& description,
                                    bool editable_key, bool editable_direction) {
  const int row = ports_table_->rowCount();
  ports_table_->insertRow(row);

  auto* key_item = new QTableWidgetItem(name);
  if (!editable_key) {
    key_item->setFlags(key_item->flags() & ~Qt::ItemIsEditable);
  }
  ports_table_->setItem(row, 0, key_item);

  if (editable_direction) {
    auto* combo = new QComboBox(ports_table_);
    combo->addItems({QStringLiteral("Input"), QStringLiteral("Output"), QStringLiteral("In/Out")});
    combo->setCurrentText(DirectionLabel(direction));
    ports_table_->setCellWidget(row, 1, combo);
    connect(combo, &QComboBox::currentTextChanged, this, &BtCustomNodeDialog::checkValid);
  } else {
    auto* direction_item = new QTableWidgetItem(DirectionLabel(direction));
    direction_item->setFlags(direction_item->flags() & ~Qt::ItemIsEditable);
    ports_table_->setItem(row, 1, direction_item);
  }

  ports_table_->setItem(row, 2, new QTableWidgetItem(default_value));
  auto* description_item = new QTableWidgetItem(description);
  if (!editable_key && name == QLatin1String(kSharedBlackboardPort)) {
    description_item->setFlags(description_item->flags() & ~Qt::ItemIsEditable);
  }
  ports_table_->setItem(row, 3, description_item);
}

void BtCustomNodeDialog::onAddPort() {
  addPortRow(QStringLiteral("key_name"), BtPortDirection::kInput, {}, {});
  checkValid();
}

void BtCustomNodeDialog::onRemovePort() {
  const auto selected = ports_table_->selectionModel()->selectedRows();
  for (int i = selected.size() - 1; i >= 0; --i) {
    ports_table_->removeRow(selected.at(i).row());
  }
  checkValid();
}

void BtCustomNodeDialog::onTypeChanged(const QString& type) {
  const bool is_subtree = type == QLatin1String("SubTree");
  const auto shared_items =
      ports_table_->findItems(QString::fromLatin1(kSharedBlackboardPort), Qt::MatchExactly);
  if (is_subtree) {
    if (shared_items.isEmpty()) {
      addPortRow(QString::fromLatin1(kSharedBlackboardPort), BtPortDirection::kInput,
                 QStringLiteral("false"),
                 tr("If false (default), the Subtree has an isolated blackboard and needs port "
                    "remapping"),
                 false, false);
    }
  } else {
    for (QTableWidgetItem* item : shared_items) {
      ports_table_->removeRow(item->row());
    }
  }
  checkValid();
}

void BtCustomNodeDialog::checkValid() {
  bool valid = false;
  const QString name = name_edit_->text().trimmed();
  QRegularExpressionValidator validator(QRegularExpression(QStringLiteral("^\\w+$")));

  if (name.compare(QStringLiteral("root"), Qt::CaseInsensitive) == 0) {
    warning_label_->setText(tr("The name 'root' is forbidden"));
  } else if (name.isEmpty()) {
    warning_label_->setText(tr("The name cannot be empty"));
  } else {
    int pos = 0;
    QString mutable_name = name;
    if (validator.validate(mutable_name, pos) != QValidator::Acceptable) {
      warning_label_->setText(
          tr("Invalid name: use only letters, digits and underscores"));
    } else if (models_.contains(name) && !editing_) {
      warning_label_->setText(tr("Another node has the same name"));
    } else {
      bool empty_param = false;
      bool invalid_param = false;
      bool reserved_param = false;
      QSet<QString> param_names;
      for (int row = 0; row < ports_table_->rowCount(); ++row) {
        const QString param_name = ports_table_->item(row, 0)->text().trimmed();
        if (param_name.isEmpty()) {
          empty_param = true;
        } else {
          QString mutable_param = param_name;
          int param_pos = 0;
          if (validator.validate(mutable_param, param_pos) != QValidator::Acceptable) {
            invalid_param = true;
          } else if (param_name == QLatin1String("ID") || param_name == QLatin1String("name")) {
            reserved_param = true;
          } else {
            param_names.insert(param_name);
          }
        }
      }
      if (empty_param) {
        warning_label_->setText(tr("Empty port name"));
      } else if (invalid_param) {
        warning_label_->setText(tr("Invalid port name: use only letters, digits and underscores."));
      } else if (reserved_param) {
        warning_label_->setText(
            tr("Reserved port key: the words \"name\" and \"ID\" should not be used."));
      } else if (param_names.size() < ports_table_->rowCount()) {
        warning_label_->setText(tr("Duplicated port name"));
      } else {
        valid = true;
      }
    }
  }

  if (valid) {
    warning_label_->setText(tr("OK"));
    warning_label_->setStyleSheet(QStringLiteral("color: rgb(78, 154, 6);"));
  } else {
    warning_label_->setStyleSheet(QStringLiteral("color: rgb(204, 0, 0);"));
  }
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(valid);
}

std::optional<BtNodeModel> BtCustomNodeDialog::model() const {
  BtNodeModel result;
  result.registration_id = name_edit_->text().trimmed();
  result.kind = KindFromComboIndex(type_combo_->currentIndex());
  for (int row = 0; row < ports_table_->rowCount(); ++row) {
    BtPortModel port;
    port.name = ports_table_->item(row, 0)->text().trimmed();
    if (auto* combo = qobject_cast<QComboBox*>(ports_table_->cellWidget(row, 1))) {
      port.direction = DirectionFromLabel(combo->currentText());
    } else {
      port.direction = DirectionFromLabel(ports_table_->item(row, 1)->text());
    }
    port.default_value = ports_table_->item(row, 2)->text();
    port.description = ports_table_->item(row, 3)->text();
    result.ports.push_back(port);
  }
  return result;
}

void BtCustomNodeDialog::closeEvent(QCloseEvent* event) {
  QSettings settings;
  settings.setValue(QStringLiteral("BtCustomNodeDialog/geometry"), saveGeometry());
  settings.setValue(QStringLiteral("BtCustomNodeDialog/header"),
                    ports_table_->horizontalHeader()->saveState());
  QDialog::closeEvent(event);
}

}  // namespace behavior_tree
}  // namespace autoviz
