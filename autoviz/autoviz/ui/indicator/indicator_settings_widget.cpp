/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_settings_widget.hpp"

#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include <QAbstractItemModel>

#include <algorithm>

#include "autoviz/common/visualization_manager.hpp"

namespace autoviz {
namespace indicator {
namespace {

QComboBox* MakeCombo(QWidget* parent) {
  auto* combo = new QComboBox(parent);
  combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  return combo;
}

QString RuleSummary(const IndicatorRule& rule, int index) {
  return QStringLiteral("%1. %2 %3 → %4")
      .arg(index + 1)
      .arg(IndicatorComparisonLabel(rule.comparison))
      .arg(rule.compare_with)
      .arg(rule.label.isEmpty() ? QStringLiteral("—") : rule.label);
}

}  // namespace

IndicatorSettingsWidget::IndicatorSettingsWidget(common::VisualizationManager* manager,
                                                 QWidget* parent)
    : manager_(manager), config_(DefaultIndicatorPanelConfig()), QWidget(parent) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  auto* general = new QGroupBox(tr("General"), this);
  auto* general_form = new QFormLayout(general);
  title_edit_ = new QLineEdit(general);
  general_form->addRow(tr("Title"), title_edit_);
  channel_combo_ = MakeCombo(general);
  channel_combo_->setEditable(true);
  field_path_edit_ = new QLineEdit(general);
  field_path_edit_->setPlaceholderText(tr("e.g. battery.percentage"));
  style_combo_ = MakeCombo(general);
  style_combo_->addItem(IndicatorStyleLabel(IndicatorStyle::kBulb),
                        static_cast<int>(IndicatorStyle::kBulb));
  style_combo_->addItem(IndicatorStyleLabel(IndicatorStyle::kBackground),
                        static_cast<int>(IndicatorStyle::kBackground));
  general_form->addRow(tr("Channel"), channel_combo_);
  general_form->addRow(tr("Field path"), field_path_edit_);
  general_form->addRow(tr("Style"), style_combo_);
  root->addWidget(general);

  auto* rules_group = new QGroupBox(tr("Rules (first match wins)"), this);
  auto* rules_layout = new QVBoxLayout(rules_group);
  auto* rule_buttons = new QHBoxLayout();
  auto* add_rule = new QPushButton(tr("Add rule"), rules_group);
  auto* remove_rule = new QPushButton(tr("Remove"), rules_group);
  rule_buttons->addWidget(add_rule);
  rule_buttons->addWidget(remove_rule);
  rule_buttons->addStretch(1);
  rules_layout->addLayout(rule_buttons);

  rules_list_ = new QListWidget(rules_group);
  rules_list_->setDragDropMode(QAbstractItemView::InternalMove);
  rules_list_->setDefaultDropAction(Qt::MoveAction);
  rules_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  rules_list_->setToolTip(tr("Drag rules to change priority (top = highest)"));
  rules_layout->addWidget(rules_list_);

  rule_editor_ = new QWidget(rules_group);
  auto* rule_form = new QFormLayout(rule_editor_);
  comparison_combo_ = MakeCombo(rule_editor_);
  comparison_combo_->addItem(IndicatorComparisonLabel(IndicatorComparison::kEqual),
                             static_cast<int>(IndicatorComparison::kEqual));
  comparison_combo_->addItem(IndicatorComparisonLabel(IndicatorComparison::kNotEqual),
                             static_cast<int>(IndicatorComparison::kNotEqual));
  comparison_combo_->addItem(IndicatorComparisonLabel(IndicatorComparison::kGreaterThan),
                             static_cast<int>(IndicatorComparison::kGreaterThan));
  comparison_combo_->addItem(
      IndicatorComparisonLabel(IndicatorComparison::kGreaterThanOrEqual),
      static_cast<int>(IndicatorComparison::kGreaterThanOrEqual));
  comparison_combo_->addItem(IndicatorComparisonLabel(IndicatorComparison::kLessThan),
                             static_cast<int>(IndicatorComparison::kLessThan));
  comparison_combo_->addItem(
      IndicatorComparisonLabel(IndicatorComparison::kLessThanOrEqual),
      static_cast<int>(IndicatorComparison::kLessThanOrEqual));
  compare_with_edit_ = new QLineEdit(rule_editor_);
  compare_with_edit_->setPlaceholderText(tr("Number, string, or true/false"));
  label_edit_ = new QLineEdit(rule_editor_);
  color_button_ = new QPushButton(tr("Pick color"), rule_editor_);
  rule_form->addRow(tr("Comparison"), comparison_combo_);
  rule_form->addRow(tr("Compare with"), compare_with_edit_);
  rule_form->addRow(tr("Label"), label_edit_);
  rule_form->addRow(tr("Color"), color_button_);
  rules_layout->addWidget(rule_editor_);
  root->addWidget(rules_group);
  root->addStretch(1);

  connect(add_rule, &QPushButton::clicked, this, &IndicatorSettingsWidget::onAddRule);
  connect(remove_rule, &QPushButton::clicked, this, &IndicatorSettingsWidget::onRemoveRule);
  connect(rules_list_, &QListWidget::currentRowChanged, this,
          &IndicatorSettingsWidget::onRuleSelectionChanged);
  connect(rules_list_->model(), &QAbstractItemModel::rowsMoved, this,
          &IndicatorSettingsWidget::onRulesReordered);
  connect(color_button_, &QPushButton::clicked, this,
          &IndicatorSettingsWidget::pickRuleColor);

  const auto wire = [this]() { emitConfigChanged(); };
  connect(title_edit_, &QLineEdit::textEdited, this, wire);
  connect(channel_combo_, &QComboBox::currentTextChanged, this, wire);
  connect(field_path_edit_, &QLineEdit::textEdited, this, wire);
  connect(style_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, wire);
  connect(comparison_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, wire);
  connect(compare_with_edit_, &QLineEdit::textEdited, this, wire);
  connect(label_edit_, &QLineEdit::textEdited, this, wire);

  setConfig(config_);
  refreshChannels();
}

IndicatorPanelConfig IndicatorSettingsWidget::config() {
  saveCurrentRuleEditor();
  config_.title = title_edit_->text().trimmed();
  config_.channel = channel_combo_->currentText().trimmed();
  config_.field_path = field_path_edit_->text().trimmed();
  config_.style = static_cast<IndicatorStyle>(style_combo_->currentData().toInt());
  return config_;
}

void IndicatorSettingsWidget::setConfig(const IndicatorPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  field_path_edit_->setText(config_.field_path);
  style_combo_->setCurrentIndex(style_combo_->findData(static_cast<int>(config_.style)));
  refreshChannels();
  const int channel_index = channel_combo_->findText(config_.channel);
  if (channel_index >= 0) {
    channel_combo_->setCurrentIndex(channel_index);
  } else {
    channel_combo_->setEditText(config_.channel);
  }
  rebuildRuleList();
}

void IndicatorSettingsWidget::refreshChannels() {
  if (manager_ == nullptr) {
    return;
  }
  const QString previous = channel_combo_->currentText();
  channel_combo_->clear();
  for (const integration::ChannelInfo& info : manager_->channels()) {
    channel_combo_->addItem(QString::fromStdString(info.channel_name));
  }
  const int index = channel_combo_->findText(previous);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  } else if (!previous.isEmpty()) {
    channel_combo_->setEditText(previous);
  }
}

void IndicatorSettingsWidget::rebuildRuleList() {
  rules_list_->clear();
  for (int i = 0; i < config_.rules.size(); ++i) {
    auto* item = new QListWidgetItem(RuleSummary(config_.rules.at(i), i), rules_list_);
    item->setData(Qt::UserRole, i);
  }
  if (config_.rules.isEmpty()) {
    selected_rule_index_ = -1;
    rule_editor_->setEnabled(false);
    return;
  }
  selected_rule_index_ =
      std::clamp(selected_rule_index_, 0, static_cast<int>(config_.rules.size()) - 1);
  rules_list_->setCurrentRow(selected_rule_index_);
  loadRuleEditor(config_.rules.at(selected_rule_index_));
  rule_editor_->setEnabled(true);
}

void IndicatorSettingsWidget::loadRuleEditor(const IndicatorRule& rule) {
  comparison_combo_->setCurrentIndex(
      comparison_combo_->findData(static_cast<int>(rule.comparison)));
  compare_with_edit_->setText(rule.compare_with);
  label_edit_->setText(rule.label);
  color_button_->setStyleSheet(
      rule.color.isValid() ? QStringLiteral("background:%1;").arg(rule.color.name())
                           : QString());
  syncComparisonUi();
}

IndicatorRule IndicatorSettingsWidget::readRuleEditor() const {
  IndicatorRule rule;
  rule.comparison =
      static_cast<IndicatorComparison>(comparison_combo_->currentData().toInt());
  rule.compare_with = compare_with_edit_->text().trimmed();
  rule.label = label_edit_->text().trimmed();
  if (selected_rule_index_ >= 0 && selected_rule_index_ < config_.rules.size()) {
    rule.color = config_.rules.at(selected_rule_index_).color;
  }
  return rule;
}

void IndicatorSettingsWidget::saveCurrentRuleEditor() {
  if (selected_rule_index_ < 0 || selected_rule_index_ >= config_.rules.size()) {
    return;
  }
  config_.rules[selected_rule_index_] = readRuleEditor();
  if (rules_list_->currentRow() == selected_rule_index_) {
    rules_list_->item(selected_rule_index_)->setText(
        RuleSummary(config_.rules.at(selected_rule_index_), selected_rule_index_));
  }
}

void IndicatorSettingsWidget::syncComparisonUi() {
  compare_with_edit_->setEnabled(true);
}

void IndicatorSettingsWidget::onAddRule() {
  saveCurrentRuleEditor();
  IndicatorRule rule;
  rule.color = QColor(120, 120, 130);
  rule.label = tr("State");
  config_.rules.push_back(rule);
  selected_rule_index_ = config_.rules.size() - 1;
  rebuildRuleList();
  emitConfigChanged();
}

void IndicatorSettingsWidget::onRemoveRule() {
  if (selected_rule_index_ < 0 || selected_rule_index_ >= config_.rules.size()) {
    return;
  }
  config_.rules.removeAt(selected_rule_index_);
  selected_rule_index_ =
      std::min(selected_rule_index_, static_cast<int>(config_.rules.size()) - 1);
  rebuildRuleList();
  emitConfigChanged();
}

void IndicatorSettingsWidget::onRuleSelectionChanged() {
  saveCurrentRuleEditor();
  selected_rule_index_ = rules_list_->currentRow();
  if (selected_rule_index_ < 0 || selected_rule_index_ >= config_.rules.size()) {
    rule_editor_->setEnabled(false);
    return;
  }
  loadRuleEditor(config_.rules.at(selected_rule_index_));
  rule_editor_->setEnabled(true);
}

void IndicatorSettingsWidget::onRulesReordered() {
  saveCurrentRuleEditor();
  QVector<IndicatorRule> reordered;
  reordered.reserve(rules_list_->count());
  for (int i = 0; i < rules_list_->count(); ++i) {
    const int index = rules_list_->item(i)->data(Qt::UserRole).toInt();
    if (index >= 0 && index < config_.rules.size()) {
      reordered.push_back(config_.rules.at(index));
    }
  }
  if (reordered.size() != config_.rules.size()) {
    return;
  }
  config_.rules = reordered;
  selected_rule_index_ = rules_list_->currentRow();
  rebuildRuleList();
  emitConfigChanged();
}

void IndicatorSettingsWidget::pickRuleColor() {
  if (selected_rule_index_ < 0 || selected_rule_index_ >= config_.rules.size()) {
    return;
  }
  const QColor picked =
      QColorDialog::getColor(config_.rules.at(selected_rule_index_).color, this,
                           tr("Rule color"));
  if (!picked.isValid()) {
    return;
  }
  config_.rules[selected_rule_index_].color = picked;
  color_button_->setStyleSheet(QStringLiteral("background:%1;").arg(picked.name()));
  emitConfigChanged();
}

void IndicatorSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace indicator
}  // namespace autoviz
