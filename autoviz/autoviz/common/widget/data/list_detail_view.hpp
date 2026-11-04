/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QPointer>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QListView;
class QSplitter;
class QStackedWidget;

namespace autoviz::widget {

// Model/view master-detail scaffold. Consumers update the detail widget from
// CurrentIndexChanged without allocating one widget per model row.
class ListDetailView final : public QWidget {
  Q_OBJECT

 public:
  explicit ListDetailView(QWidget *parent = nullptr);

  void SetModel(QAbstractItemModel *model);
  QAbstractItemModel *model() const noexcept { return model_; }
  QListView *list_view() const noexcept { return list_view_; }
  QModelIndex current_index() const;
  void SetCurrentIndex(const QModelIndex &index);

  void SetDetailWidget(QWidget *widget);
  QWidget *detail_widget() const noexcept { return detail_widget_.data(); }
  QWidget *TakeDetailWidget();
  void SetPlaceholderWidget(QWidget *widget);
  QWidget *placeholder_widget() const noexcept {
    return placeholder_widget_.data();
  }
  void SetOrientation(Qt::Orientation orientation);
  Qt::Orientation orientation() const;
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 signals:
  void CurrentIndexChanged(const QModelIndex &current);

 private:
  void ConnectSelectionModel();
  void SyncPage(const QModelIndex &current);

  QSplitter *splitter_ = nullptr;
  QListView *list_view_ = nullptr;
  QStackedWidget *pages_ = nullptr;
  QPointer<QAbstractItemModel> model_;
  QPointer<QWidget> detail_widget_;
  QPointer<QWidget> placeholder_widget_;
  QMetaObject::Connection selection_connection_;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
