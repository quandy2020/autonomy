/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QKeySequence>
#include <QWidget>

class QTableWidget;

namespace autoviz {

class ShortcutsEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ShortcutsEditorWidget(QWidget* parent = nullptr);

  void setShortcuts(const QHash<QString, QKeySequence>& shortcuts);
  QHash<QString, QKeySequence> shortcuts() const;
  void resetToDefaults();

 private:
  void rebuildRows();

  QTableWidget* table_ = nullptr;
  QHash<QString, QKeySequence> shortcuts_;
};

QString TranslateShortcutLabel(const QString& shortcut_id);

}  // namespace autoviz
