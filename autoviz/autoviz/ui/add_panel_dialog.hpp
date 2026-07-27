/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDialog>
#include <QStringList>

class QListWidget;

namespace autoviz {

class AddPanelDialog : public QDialog {
  Q_OBJECT

 public:
  explicit AddPanelDialog(const QStringList& available_panels,
                          QWidget* parent = nullptr);

  QString selectedPanelObjectName() const;

 private:
  void populate(const QStringList& available_panels);

  QListWidget* list_ = nullptr;
};

}  // namespace autoviz
