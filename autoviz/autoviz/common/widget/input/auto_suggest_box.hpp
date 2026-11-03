/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QLineEdit>
#include <QMetaObject>
#include <QModelIndex>
#include <QPointer>
#include <QVariant>
#include <QVector>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAbstractItemModel;
class QFocusEvent;
class QFrame;
class QHideEvent;
class QKeyEvent;
class QListView;
class QMoveEvent;
class QResizeEvent;
class QTimer;

namespace autoviz::widget {

// Model-backed asynchronous suggestion field. The model remains owned by the
// caller; QueryRequested lets a controller refresh it without copying rows.
class AutoSuggestBox final : public QLineEdit {
  Q_OBJECT

 public:
  explicit AutoSuggestBox(QWidget* parent = nullptr);

  void SetModel(QAbstractItemModel* model);
  QAbstractItemModel* model() const noexcept { return model_.data(); }
  void SetDebounceMs(int debounce_ms);
  int debounce_ms() const noexcept { return debounce_ms_; }
  void SetMaximumVisibleItems(int maximum_items);
  int maximum_visible_items() const noexcept { return maximum_visible_items_; }
  void SetDataRole(int data_role);
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;

 signals:
  void QueryRequested(const QString& query);
  void SuggestionActivated(const QModelIndex& index, const QVariant& data);

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void moveEvent(QMoveEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  void ActivateIndex(const QModelIndex& index);
  void DisconnectModelSignals();
  void HideSuggestions();
  void InstallPositionWatchers();
  void MoveCurrent(int offset);
  void RemovePositionWatchers();
  void RepositionPopup();
  void ScheduleQuery(const QString& query);
  void ShowSuggestions();

  QPointer<QAbstractItemModel> model_;
  QFrame* popup_ = nullptr;
  QListView* list_view_ = nullptr;
  QTimer* debounce_timer_ = nullptr;
  QMetaObject::Connection model_reset_connection_;
  QMetaObject::Connection rows_inserted_connection_;
  QMetaObject::Connection rows_removed_connection_;
  QMetaObject::Connection model_destroyed_connection_;
  QVector<QPointer<QObject>> position_watchers_;
  style::Tone tone_ = style::Tone::Shell;
  int debounce_ms_ = 180;
  int maximum_visible_items_ = 8;
  int data_role_ = Qt::UserRole;
};

}  // namespace autoviz::widget
