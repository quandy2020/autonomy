/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <optional>
#include <vector>

#include "autolink/parameter/parameter.hpp"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QLabel;
class QTableWidget;
class QTableWidgetItem;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace parameters_panel {

struct ParametersPanelConfig {
  QString service_node;
  QString filter;
  bool auto_refresh = true;
};

ParametersPanelConfig DefaultParametersPanelConfig();

class ParametersPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ParametersPanel(common::VisualizationManager* manager,
                           QWidget* parent = nullptr);

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

  ParametersPanelConfig config() const;
  void setConfig(const ParametersPanelConfig& config);
  void cloneConfigFrom(const ParametersPanelConfig& config);

 public slots:
  void refreshNodeList();
  void refreshParameters();

 signals:
  void activated();
  void configChanged();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private slots:
  void onNodeSelectionChanged(int index);
  void onFilterChanged(const QString& text);
  void onAutoRefreshToggled(bool enabled);
  void onParameterItemChanged(QTableWidgetItem* item);
  void onRefreshClicked();

 private:
  void setupUi();
  void applyConfigToUi();
  void syncConfigFromUi();
  void populateNodeCombo(const std::vector<std::string>& nodes);
  void rebuildParameterTable(const std::vector<autolink::Parameter>& parameters);
  void applyFilter();
  void updateStatusText(const QString& text, bool is_error = false);
  bool loadParametersForCurrentNode(std::vector<autolink::Parameter>* parameters,
                                    QString* error_message);
  bool commitParameterEdit(int row, const QString& new_text);

  common::VisualizationManager* manager_ = nullptr;
  ParametersPanelConfig config_;
  QComboBox* node_combo_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QCheckBox* auto_refresh_check_ = nullptr;
  QToolButton* refresh_button_ = nullptr;
  QLabel* status_label_ = nullptr;
  QTableWidget* table_ = nullptr;
  QTimer* node_timer_ = nullptr;
  QTimer* parameter_timer_ = nullptr;
  QTimer* filter_timer_ = nullptr;
  bool suppress_table_signals_ = false;
  bool loading_parameters_ = false;
  QToolButton* expand_button_ = nullptr;
};

}  // namespace parameters_panel
}  // namespace autoviz
