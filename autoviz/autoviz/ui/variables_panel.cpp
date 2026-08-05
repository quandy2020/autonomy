/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/variables_panel.hpp"

#include <optional>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <functional>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/variables/variable_store.hpp"
#include "autoviz/variables/variable_types.hpp"

namespace autoviz {
namespace {

using variables::VariableEntry;
using variables::VariableStore;
using variables::VariableType;

class NumberEditor : public QDoubleSpinBox {
 public:
  using StepCallback = std::function<void(double)>;

  explicit NumberEditor(QWidget* parent = nullptr) : QDoubleSpinBox(parent) {
    setDecimals(6);
    setRange(-1e12, 1e12);
    setSingleStep(1.0);
    setKeyboardTracking(false);
  }

  void setStepCallback(StepCallback callback) { step_callback_ = std::move(callback); }

 protected:
  void keyPressEvent(QKeyEvent* event) override {
    if (step_callback_) {
      if (event->key() == Qt::Key_Up) {
        step_callback_(singleStep());
        event->accept();
        return;
      }
      if (event->key() == Qt::Key_Down) {
        step_callback_(-singleStep());
        event->accept();
        return;
      }
    }
    QDoubleSpinBox::keyPressEvent(event);
  }

 private:
  StepCallback step_callback_;
};

VariableType TypeFromComboIndex(int index) {
  switch (index) {
    case 1:
      return VariableType::kNumber;
    case 2:
      return VariableType::kBoolean;
    case 3:
      return VariableType::kArray;
    case 4:
      return VariableType::kMap;
    default:
      return VariableType::kString;
  }
}

int ComboIndexFromType(VariableType type) {
  switch (type) {
    case VariableType::kNumber:
      return 1;
    case VariableType::kBoolean:
      return 2;
    case VariableType::kArray:
      return 3;
    case VariableType::kMap:
      return 4;
    default:
      return 0;
  }
}

QString DefaultNameForType(VariableType type) {
  switch (type) {
    case VariableType::kNumber:
      return QStringLiteral("my_number");
    case VariableType::kBoolean:
      return QStringLiteral("my_flag");
    case VariableType::kArray:
      return QStringLiteral("my_array");
    case VariableType::kMap:
      return QStringLiteral("my_map");
    default:
      return QStringLiteral("my_string");
  }
}

QVariant DefaultValueForType(VariableType type) {
  switch (type) {
    case VariableType::kNumber:
      return 0.0;
    case VariableType::kBoolean:
      return false;
    case VariableType::kArray:
      return QStringLiteral("[]");
    case VariableType::kMap:
      return QStringLiteral("{}");
    default:
      return QString{};
  }
}

}  // namespace

VariablesPanel::VariablesPanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : manager_(manager), QWidget(parent) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto* hint = new QLabel(
      tr("Define global variables referenced in message paths as $name. "
         "Example: objects[:]{id==$vehicle_id}.speed"),
      this);
  hint->setWordWrap(true);
  hint->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 11px;"));
  root->addWidget(hint);

  table_ = new QTableWidget(this);
  table_->setColumnCount(3);
  table_->setHorizontalHeaderLabels(
      {tr("Name"), tr("Type"), tr("Value")});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->verticalHeader()->setVisible(false);
  root->addWidget(table_, 1);

  auto* buttons = new QHBoxLayout();
  add_button_ = new QPushButton(tr("Add variable"), this);
  remove_button_ = new QPushButton(tr("Remove"), this);
  buttons->addWidget(add_button_);
  buttons->addWidget(remove_button_);
  buttons->addStretch();
  root->addLayout(buttons);

  connect(add_button_, &QPushButton::clicked, this, &VariablesPanel::onAddVariable);
  connect(remove_button_, &QPushButton::clicked, this,
          &VariablesPanel::onRemoveSelected);
  connect(table_, &QTableWidget::cellChanged, this, &VariablesPanel::onCellChanged);

  if (manager_ != nullptr) {
    connect(&manager_->variableStore(), &VariableStore::variablesChanged, this,
            &VariablesPanel::onStoreChanged);
    rebuildTable();
  }
}

void VariablesPanel::rebuildTable() {
  if (manager_ == nullptr || table_ == nullptr) {
    return;
  }
  syncing_ = true;
  table_->setRowCount(0);
  const QVector<VariableEntry> entries = manager_->variableStore().variables();
  table_->setRowCount(entries.size());
  for (int row = 0; row < entries.size(); ++row) {
    syncRowFromStore(row);
  }
  syncing_ = false;
}

void VariablesPanel::syncRowFromStore(int row) {
  if (manager_ == nullptr || table_ == nullptr) {
    return;
  }
  const QVector<VariableEntry> entries = manager_->variableStore().variables();
  if (row < 0 || row >= entries.size()) {
    return;
  }
  const VariableEntry& entry = entries[row];

  auto* name_item = table_->item(row, 0);
  if (name_item == nullptr) {
    name_item = new QTableWidgetItem();
    table_->setItem(row, 0, name_item);
  }
  name_item->setText(entry.name);

  auto* type_combo = qobject_cast<QComboBox*>(table_->cellWidget(row, 1));
  if (type_combo == nullptr) {
    type_combo = new QComboBox(table_);
    type_combo->addItems({tr("string"), tr("number"), tr("boolean"),
                          tr("array"), tr("map")});
    connect(type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this, row](int index) {
              if (syncing_ || manager_ == nullptr) {
                return;
              }
              QTableWidgetItem* name_item = table_->item(row, 0);
              if (name_item == nullptr) {
                return;
              }
              const QString name = name_item->text().trimmed();
              if (!variables::IsValidVariableName(name)) {
                return;
              }
              const VariableType type = TypeFromComboIndex(index);
              manager_->variableStore().setVariable(name, type,
                                                    DefaultValueForType(type));
            });
    table_->setCellWidget(row, 1, type_combo);
  }
  type_combo->blockSignals(true);
  type_combo->setCurrentIndex(ComboIndexFromType(entry.type));
  type_combo->blockSignals(false);

  QWidget* existing = table_->cellWidget(row, 2);
  if (existing != nullptr) {
    table_->removeCellWidget(row, 2);
    existing->deleteLater();
  }
  table_->setCellWidget(row, 2, createValueEditor(row, entry.name));
}

QWidget* VariablesPanel::createValueEditor(int row, const QString& name) {
  const std::optional<VariableEntry> entry =
      manager_->variableStore().variable(name);
  if (!entry.has_value()) {
    return new QWidget(this);
  }

  switch (entry->type) {
    case VariableType::kNumber: {
      auto* editor = new NumberEditor(table_);
      editor->setValue(entry->value.toDouble());
      connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [this, row](double) {
                if (syncing_) {
                  return;
                }
                applyValueEditor(row);
              });
      editor->setStepCallback([this, row](double delta) {
        handleNumberStep(row, delta);
      });
      return editor;
    }
    case VariableType::kBoolean: {
      auto* editor = new QCheckBox(table_);
      editor->setChecked(entry->value.toBool());
      connect(editor, &QCheckBox::toggled, this, [this, row](bool) {
        if (syncing_) {
          return;
        }
        applyValueEditor(row);
      });
      return editor;
    }
    case VariableType::kArray:
    case VariableType::kMap: {
      auto* editor = new QPlainTextEdit(table_);
      editor->setPlaceholderText(entry->type == VariableType::kArray
                                     ? QStringLiteral("[1, 2, 3]")
                                     : QStringLiteral("{\"key\": \"value\"}"));
      editor->setPlainText(
          variables::SerializeVariableValue(entry->type, entry->value));
      editor->setMaximumHeight(72);
      connect(editor, &QPlainTextEdit::textChanged, this, [this, row]() {
        if (syncing_) {
          return;
        }
        applyValueEditor(row);
      });
      return editor;
    }
    default: {
      auto* editor = new QLineEdit(table_);
      editor->setText(entry->value.toString());
      connect(editor, &QLineEdit::editingFinished, this, [this, row]() {
        if (syncing_) {
          return;
        }
        applyValueEditor(row);
      });
      return editor;
    }
  }
}

void VariablesPanel::applyValueEditor(int row) {
  if (manager_ == nullptr || syncing_ || row < 0 ||
      row >= manager_->variableStore().variables().size()) {
    return;
  }
  const QVector<VariableEntry> entries = manager_->variableStore().variables();
  const QString old_name = entries[row].name;

  QTableWidgetItem* name_item = table_->item(row, 0);
  const QString new_name =
      name_item != nullptr ? name_item->text().trimmed() : old_name;
  auto* type_combo = qobject_cast<QComboBox*>(table_->cellWidget(row, 1));
  const VariableType type =
      type_combo != nullptr ? TypeFromComboIndex(type_combo->currentIndex())
                            : entries[row].type;

  QVariant value = DefaultValueForType(type);
  if (QWidget* editor = table_->cellWidget(row, 2)) {
    if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) {
      value = spin->value();
    } else if (auto* check = qobject_cast<QCheckBox*>(editor)) {
      value = check->isChecked();
    } else if (auto* line = qobject_cast<QLineEdit*>(editor)) {
      value = line->text();
    } else if (auto* json = qobject_cast<QPlainTextEdit*>(editor)) {
      value = json->toPlainText();
    }
  }

  VariableStore& store = manager_->variableStore();
  if (new_name != old_name) {
    if (!variables::IsValidVariableName(new_name) ||
        store.variable(new_name).has_value()) {
      syncing_ = true;
      if (name_item != nullptr) {
        name_item->setText(old_name);
      }
      syncing_ = false;
      return;
    }
    store.removeVariable(old_name);
  }
  store.setVariable(new_name, type, value);
}

void VariablesPanel::handleNumberStep(int row, double delta) {
  if (manager_ == nullptr || row < 0) {
    return;
  }
  const QVector<VariableEntry> entries = manager_->variableStore().variables();
  if (row >= entries.size()) {
    return;
  }
  manager_->variableStore().adjustNumber(entries[row].name, delta);
  if (QWidget* editor = table_->cellWidget(row, 2)) {
    if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) {
      syncing_ = true;
      spin->setValue(manager_->variableStore().variable(entries[row].name)
                         ->value.toDouble());
      syncing_ = false;
    }
  }
}

void VariablesPanel::onAddVariable() {
  if (manager_ == nullptr) {
    return;
  }
  VariableStore& store = manager_->variableStore();
  VariableType type = VariableType::kString;
  QString name = DefaultNameForType(type);
  int suffix = 1;
  while (store.variable(name).has_value()) {
    name = DefaultNameForType(type) + QString::number(suffix++);
  }
  store.setVariable(name, type, DefaultValueForType(type));
}

void VariablesPanel::onRemoveSelected() {
  if (manager_ == nullptr || table_ == nullptr) {
    return;
  }
  const int row = table_->currentRow();
  if (row < 0) {
    return;
  }
  const QVector<VariableEntry> entries = manager_->variableStore().variables();
  if (row >= entries.size()) {
    return;
  }
  manager_->variableStore().removeVariable(entries[row].name);
}

void VariablesPanel::onStoreChanged() {
  rebuildTable();
}

void VariablesPanel::onCellChanged(int row, int column) {
  if (syncing_ || column != 0) {
    return;
  }
  applyValueEditor(row);
}

}  // namespace autoviz
