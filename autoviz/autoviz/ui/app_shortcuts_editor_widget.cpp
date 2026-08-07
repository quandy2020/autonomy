/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app_shortcuts_editor_widget.hpp"

#include <QAbstractItemView>
#include <QHeaderView>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QTableWidget>
#include <QVBoxLayout>

#include "autoviz/ui/app_preferences.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

QString TranslateShortcutLabel(const QString& shortcut_id) {
  if (shortcut_id == QLatin1String("file.open")) {
    return ShortcutsEditorWidget::tr("Open config");
  }
  if (shortcut_id == QLatin1String("file.save")) {
    return ShortcutsEditorWidget::tr("Save config");
  }
  if (shortcut_id == QLatin1String("file.save_as")) {
    return ShortcutsEditorWidget::tr("Save config as");
  }
  if (shortcut_id == QLatin1String("file.quit")) {
    return ShortcutsEditorWidget::tr("Quit");
  }
  if (shortcut_id == QLatin1String("panels.fullscreen")) {
    return ShortcutsEditorWidget::tr("Toggle fullscreen");
  }
  if (shortcut_id == QLatin1String("tools.reset")) {
    return ShortcutsEditorWidget::tr("Reset active tool");
  }
  return shortcut_id;
}

ShortcutsEditorWidget::ShortcutsEditorWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);

  table_ = new QTableWidget(this);
  table_->setColumnCount(2);
  table_->setHorizontalHeaderLabels({tr("Action"), tr("Shortcut")});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table_->verticalHeader()->setVisible(false);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setAlternatingRowColors(true);
  table_->setShowGrid(false);
  table_->setMinimumHeight(180);
  layout->addWidget(table_);

  auto* hint = new QLabel(
      tr("Click a shortcut field and press the desired key combination. "
         "Tool letter shortcuts are configured from the toolbar."),
      this);
  hint->setWordWrap(true);
  StyleHintLabel(hint);
  layout->addWidget(hint);

  for (const AppShortcutDefinition& definition : DefaultShortcutDefinitions()) {
    shortcuts_.insert(definition.id, definition.default_sequence);
  }
  rebuildRows();
}

void ShortcutsEditorWidget::setShortcuts(const QHash<QString, QKeySequence>& shortcuts) {
  shortcuts_ = shortcuts;
  for (const AppShortcutDefinition& definition : DefaultShortcutDefinitions()) {
    if (!shortcuts_.contains(definition.id)) {
      shortcuts_.insert(definition.id, definition.default_sequence);
    }
  }
  rebuildRows();
}

QHash<QString, QKeySequence> ShortcutsEditorWidget::shortcuts() const {
  QHash<QString, QKeySequence> result = shortcuts_;
  if (table_ == nullptr) {
    return result;
  }
  for (int row = 0; row < table_->rowCount(); ++row) {
    QTableWidgetItem* id_item = table_->item(row, 0);
    QWidget* editor = table_->cellWidget(row, 1);
    auto* key_edit = qobject_cast<QKeySequenceEdit*>(editor);
    if (id_item == nullptr || key_edit == nullptr) {
      continue;
    }
    result.insert(id_item->data(Qt::UserRole).toString(), key_edit->keySequence());
  }
  return result;
}

void ShortcutsEditorWidget::resetToDefaults() {
  shortcuts_.clear();
  for (const AppShortcutDefinition& definition : DefaultShortcutDefinitions()) {
    shortcuts_.insert(definition.id, definition.default_sequence);
  }
  rebuildRows();
}

void ShortcutsEditorWidget::rebuildRows() {
  if (table_ == nullptr) {
    return;
  }
  table_->setRowCount(0);
  const QVector<AppShortcutDefinition> definitions = DefaultShortcutDefinitions();
  table_->setRowCount(definitions.size());
  for (int row = 0; row < definitions.size(); ++row) {
    const AppShortcutDefinition& definition = definitions[row];
    auto* label_item =
        new QTableWidgetItem(TranslateShortcutLabel(definition.id));
    label_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    label_item->setData(Qt::UserRole, definition.id);
    table_->setItem(row, 0, label_item);

    auto* key_edit = new QKeySequenceEdit(
        shortcuts_.value(definition.id, definition.default_sequence), table_);
    table_->setCellWidget(row, 1, key_edit);
    table_->setRowHeight(row, 30);
  }
}

}  // namespace autoviz
