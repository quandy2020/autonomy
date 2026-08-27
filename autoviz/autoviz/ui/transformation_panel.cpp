/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/transformation_panel.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QVBoxLayout>

#include "autoviz/common/transformation_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

TransformationPanel::TransformationPanel(
    common::TransformationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setupUi();
  if (manager_ != nullptr) {
    pending_transformer_id_ = manager_->currentTransformerId();
  }
  populateList();
}

void TransformationPanel::setupUi() {
  ApplyPanelShell(this);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  list_ = new QListWidget(this);
  list_->setObjectName(QStringLiteral("TransformationPanel/TransformerList"));
  layout->addWidget(list_, 1);

  auto* button_bar = new QFrame(this);
  ApplyPanelFooterChrome(button_bar);
  auto* button_row = new QHBoxLayout(button_bar);
  button_row->setContentsMargins(PanelChromeLayout::kFooterMarginH,
                                 PanelChromeLayout::kFooterMarginV,
                                 PanelChromeLayout::kFooterMarginH,
                                 PanelChromeLayout::kFooterMarginV);
  save_button_ = MakeFlatActionButton(tr("Save"), button_bar);
  save_button_->setToolTip(
      tr("Apply the selected transformation plugin (RViz-style)."));
  save_button_->setEnabled(false);
  button_row->addStretch();
  button_row->addWidget(save_button_);
  layout->addWidget(button_bar);

  connect(list_, &QListWidget::itemChanged, this,
          &TransformationPanel::onItemChanged);
  connect(save_button_, &QPushButton::clicked, this,
          &TransformationPanel::onSaveClicked);
}

void TransformationPanel::populateList() {
  if (manager_ == nullptr || list_ == nullptr) {
    return;
  }
  updating_ = true;
  list_->clear();
  const std::string active_id = manager_->currentTransformerId();
  pending_transformer_id_ = active_id;
  for (const common::PluginInfo& info : manager_->availableTransformers()) {
    auto* item = new QListWidgetItem(QString::fromStdString(info.name), list_);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable |
                   Qt::ItemIsSelectable);
    item->setCheckState(info.class_id == active_id ? Qt::Checked
                                                   : Qt::Unchecked);
    item->setData(Qt::UserRole, QString::fromStdString(info.class_id));
    item->setToolTip(QString::fromStdString(info.description));
  }
  updating_ = false;
  updateSaveButton();
}

void TransformationPanel::refresh() { populateList(); }

void TransformationPanel::updateSaveButton() {
  if (save_button_ == nullptr || manager_ == nullptr) {
    return;
  }
  save_button_->setEnabled(pending_transformer_id_ !=
                           manager_->currentTransformerId());
}

void TransformationPanel::onItemChanged() {
  if (updating_ || list_ == nullptr) {
    return;
  }
  QListWidgetItem* changed = nullptr;
  for (int i = 0; i < list_->count(); ++i) {
    QListWidgetItem* item = list_->item(i);
    if (item->checkState() == Qt::Checked) {
      changed = item;
      break;
    }
  }
  if (changed == nullptr) {
    return;
  }

  updating_ = true;
  for (int i = 0; i < list_->count(); ++i) {
    QListWidgetItem* item = list_->item(i);
    if (item != changed) {
      item->setCheckState(Qt::Unchecked);
    }
  }
  updating_ = false;

  pending_transformer_id_ =
      changed->data(Qt::UserRole).toString().toStdString();
  updateSaveButton();
}

void TransformationPanel::onSaveClicked() {
  if (manager_ == nullptr || pending_transformer_id_.empty()) {
    return;
  }
  manager_->setTransformer(pending_transformer_id_);
  populateList();
  emit transformerChanged();
  emit configChanged();
}

}  // namespace autoviz
