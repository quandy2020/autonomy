/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QWidget>

#include "autoviz/common/visualization_manager.hpp"

namespace autoviz {

class ToolPropertiesPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ToolPropertiesPanel(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  void refresh();

 signals:
  void propertiesChanged();

 private slots:
  void onPropertyEdited();

 private:
  void setupUi();
  void populateProperties();

  std::shared_ptr<common::VisualizationManager> manager_;
  QLabel* tool_label_ = nullptr;
  QFormLayout* property_form_ = nullptr;
  QWidget* property_container_ = nullptr;
  std::vector<QLineEdit*> property_edits_;
  bool updating_ = false;
};

}  // namespace autoviz
