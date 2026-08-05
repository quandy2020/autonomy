/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/table/table_types.hpp"

class QComboBox;
class QLineEdit;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace table {

class TableSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TableSettingsWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  TablePanelConfig config() const;
  void setConfig(const TablePanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();

 private:
  void emitConfigChanged();

  common::VisualizationManager* manager_ = nullptr;
  TablePanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QLineEdit* array_path_edit_ = nullptr;
};

}  // namespace table
}  // namespace autoviz
