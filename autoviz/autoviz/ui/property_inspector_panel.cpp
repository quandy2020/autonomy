/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/property_inspector_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

PropertyInspectorPanel::PropertyInspectorPanel(QWidget* parent) : QWidget(parent) {
  ApplyPanelShell(this);
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  title_label_ = new QLabel(tr("Panel"), this);
  title_label_->setStyleSheet(PropertyInspectorTitleStyle());
  root->addWidget(title_label_);

  content_host_ = new QWidget(this);
  content_layout_ = new QVBoxLayout(content_host_);
  content_layout_->setContentsMargins(0, 0, 0, 0);
  content_layout_->setSpacing(0);
  root->addWidget(content_host_, 1);

  placeholder_ = new QWidget(content_host_);
  auto* placeholder_layout = new QVBoxLayout(placeholder_);
  auto* hint = new QLabel(
      tr("Select a panel in the center area to edit its properties here."),
      placeholder_);
  hint->setWordWrap(true);
  hint->setStyleSheet(PropertyInspectorHintStyle());
  placeholder_layout->addWidget(hint);
  placeholder_layout->addStretch(1);
  content_layout_->addWidget(placeholder_);
}

void PropertyInspectorPanel::clearContentLayout() {
  if (content_layout_ == nullptr) {
    return;
  }
  while (QLayoutItem* item = content_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->setParent(this);
      item->widget()->hide();
    }
    delete item;
  }
}

void PropertyInspectorPanel::setContentWidget(QWidget* widget,
                                              const QString& title) {
  if (widget == nullptr) {
    clearContent();
    return;
  }
  if (content_widget_ == widget) {
    title_label_->setText(title.isEmpty() ? tr("Panel") : title);
    widget->show();
    return;
  }

  if (content_widget_ != nullptr && content_widget_ != placeholder_) {
    content_widget_->hide();
  }

  clearContentLayout();
  content_widget_ = widget;
  title_label_->setText(title.isEmpty() ? tr("Panel") : title);
  content_layout_->addWidget(widget, 1);
  widget->show();
}

void PropertyInspectorPanel::clearContent() {
  if (content_widget_ != nullptr && content_widget_ != placeholder_) {
    content_widget_->hide();
  }
  content_widget_ = nullptr;
  showPlaceholder();
}

void PropertyInspectorPanel::showPlaceholder() {
  title_label_->setText(tr("Panel"));
  clearContentLayout();
  content_layout_->addWidget(placeholder_);
  placeholder_->show();
  content_widget_ = nullptr;
}

}  // namespace autoviz
