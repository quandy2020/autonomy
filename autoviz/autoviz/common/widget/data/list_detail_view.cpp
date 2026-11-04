/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/list_detail_view.hpp"

#include <QAbstractItemModel>
#include <QItemSelectionModel>
#include <QLabel>
#include <QListView>
#include <QSplitter>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

ListDetailView::ListDetailView(QWidget *parent)
    : QWidget(parent),
      splitter_(new QSplitter(this)),
      list_view_(new QListView(splitter_)),
      pages_(new QStackedWidget(splitter_)) {
  setObjectName(QStringLiteral("AutovizListDetailView"));
  setProperty("autovizRole", QStringLiteral("listDetailView"));
  setAccessibleName(tr("List and details"));

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(splitter_);
  splitter_->setChildrenCollapsible(false);
  splitter_->setStretchFactor(0, 1);
  splitter_->setStretchFactor(1, 3);
  list_view_->setAccessibleName(tr("Items"));

  auto *placeholder = new QLabel(tr("Select an item"), pages_);
  static_cast<QLabel *>(placeholder)->setAlignment(Qt::AlignCenter);
  SetPlaceholderWidget(placeholder);
  ApplyWidgetTheme(this, tone_, true);
}

void ListDetailView::SetModel(QAbstractItemModel *model) {
  if (model_ == model) {
    return;
  }
  model_ = model;
  list_view_->setModel(model);
  ConnectSelectionModel();
  SyncPage({});
}

QModelIndex ListDetailView::current_index() const {
  return list_view_->currentIndex();
}

void ListDetailView::SetCurrentIndex(const QModelIndex &index) {
  list_view_->setCurrentIndex(index);
}

void ListDetailView::SetDetailWidget(QWidget *widget) {
  if (detail_widget_ == widget) {
    return;
  }
  if (detail_widget_ != nullptr) {
    pages_->removeWidget(detail_widget_);
    detail_widget_->hide();
    detail_widget_->deleteLater();
  }
  detail_widget_ = widget;
  if (detail_widget_ != nullptr) {
    detail_widget_->setParent(pages_);
    pages_->addWidget(detail_widget_);
    ApplyWidgetTheme(detail_widget_, tone_, true);
  }
  SyncPage(current_index());
}

QWidget *ListDetailView::TakeDetailWidget() {
  QWidget *widget = detail_widget_.data();
  if (widget == nullptr) {
    return nullptr;
  }
  pages_->removeWidget(widget);
  widget->hide();
  widget->setParent(nullptr);
  detail_widget_ = nullptr;
  SyncPage(current_index());
  return widget;
}

void ListDetailView::SetPlaceholderWidget(QWidget *widget) {
  if (widget == nullptr || placeholder_widget_ == widget) {
    return;
  }
  if (placeholder_widget_ != nullptr) {
    pages_->removeWidget(placeholder_widget_);
    placeholder_widget_->deleteLater();
  }
  placeholder_widget_ = widget;
  placeholder_widget_->setParent(pages_);
  placeholder_widget_->setAccessibleName(tr("No item selected"));
  pages_->insertWidget(0, placeholder_widget_);
  ApplyWidgetTheme(placeholder_widget_, tone_, true);
  SyncPage(current_index());
}

void ListDetailView::SetOrientation(Qt::Orientation orientation) {
  splitter_->setOrientation(orientation);
}

Qt::Orientation ListDetailView::orientation() const {
  return splitter_->orientation();
}

void ListDetailView::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyWidgetTheme(this, tone_, true);
}

void ListDetailView::ConnectSelectionModel() {
  disconnect(selection_connection_);
  QItemSelectionModel *selection = list_view_->selectionModel();
  if (selection == nullptr) {
    return;
  }
  selection_connection_ =
      connect(selection, &QItemSelectionModel::currentChanged, this,
              [this](const QModelIndex &current, const QModelIndex &) {
                SyncPage(current);
                emit CurrentIndexChanged(current);
              });
}

void ListDetailView::SyncPage(const QModelIndex &current) {
  const bool show_detail = current.isValid() && detail_widget_ != nullptr;
  QWidget *page =
      show_detail ? detail_widget_.data() : placeholder_widget_.data();
  if (page != nullptr) {
    pages_->setCurrentWidget(page);
  }
  setAccessibleDescription(
      show_detail ? tr("Showing details for row %1").arg(current.row() + 1)
                  : tr("No item selected"));
}

}  // namespace autoviz::widget
