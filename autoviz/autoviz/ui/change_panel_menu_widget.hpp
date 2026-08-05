/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLineEdit;
class QListWidget;

namespace autoviz {

/** Searchable panel picker (Foxglove Change panel submenu). */
class ChangePanelMenuWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ChangePanelMenuWidget(QWidget* parent = nullptr);

 signals:
  void panelSelected(const QString& object_name);

 private slots:
  void onFilterChanged(const QString& text);
  void onItemActivated();

 private:
  void populate();

  QLineEdit* search_ = nullptr;
  QListWidget* list_ = nullptr;
};

}  // namespace autoviz
