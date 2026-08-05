/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;
class QCheckBox;

namespace autoviz {

namespace common {
class VisualizationManager;
}

/** Foxglove-style global Variables sidebar for message path expressions. */
class VariablesPanel : public QWidget {
  Q_OBJECT

 public:
  explicit VariablesPanel(common::VisualizationManager* manager,
                          QWidget* parent = nullptr);

 private slots:
  void onAddVariable();
  void onRemoveSelected();
  void onStoreChanged();
  void onCellChanged(int row, int column);

 private:
  void rebuildTable();
  void syncRowFromStore(int row);
  QWidget* createValueEditor(int row, const QString& name);
  void applyValueEditor(int row);
  void handleNumberStep(int row, double delta);

  common::VisualizationManager* manager_ = nullptr;
  QTableWidget* table_ = nullptr;
  QPushButton* add_button_ = nullptr;
  QPushButton* remove_button_ = nullptr;
  bool syncing_ = false;
};

}  // namespace autoviz
