/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/indicator/indicator_types.hpp"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace indicator {

class IndicatorSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit IndicatorSettingsWidget(common::VisualizationManager* manager,
                                   QWidget* parent = nullptr);

  IndicatorPanelConfig config();
  void setConfig(const IndicatorPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();

 private slots:
  void onAddRule();
  void onRemoveRule();
  void onRuleSelectionChanged();
  void onRulesReordered();
  void emitConfigChanged();
  void pickRuleColor();

 private:
  void rebuildRuleList();
  void loadRuleEditor(const IndicatorRule& rule);
  IndicatorRule readRuleEditor() const;
  void saveCurrentRuleEditor();
  void syncComparisonUi();

  common::VisualizationManager* manager_ = nullptr;
  IndicatorPanelConfig config_;
  int selected_rule_index_ = -1;

  QLineEdit* title_edit_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QLineEdit* field_path_edit_ = nullptr;
  QComboBox* style_combo_ = nullptr;

  QListWidget* rules_list_ = nullptr;
  QComboBox* comparison_combo_ = nullptr;
  QLineEdit* compare_with_edit_ = nullptr;
  QLineEdit* label_edit_ = nullptr;
  QPushButton* color_button_ = nullptr;
  QWidget* rule_editor_ = nullptr;
};

}  // namespace indicator
}  // namespace autoviz
