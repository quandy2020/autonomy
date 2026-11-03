/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDialog>
#include <QModelIndex>
#include <QPointer>
#include <QVariant>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAbstractItemModel;
class QEvent;
class QListView;
class QSortFilterProxyModel;

namespace autoviz::widget {

class SearchBox;

// Keyboard-first command selector backed by a caller-owned item model.
class CommandPalette final : public QDialog {
  Q_OBJECT

 public:
  enum class MatchMode {
    kContains,
    kFuzzy,
  };

  explicit CommandPalette(QWidget* parent = nullptr);

  void SetModel(QAbstractItemModel* model);
  QAbstractItemModel* model() const noexcept { return model_.data(); }
  void SetMatchMode(MatchMode mode);
  MatchMode match_mode() const noexcept { return match_mode_; }
  void SetRoles(int id_role, int data_role);
  void SetTone(style::Tone tone);

  void ShowPalette();
  void Dismiss();

  QSize sizeHint() const override;

 signals:
  void CommandActivated(const QString& command_id, const QVariant& data);
  void SourceIndexActivated(const QModelIndex& source_index);

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;
  void showEvent(QShowEvent* event) override;

 private:
  class CommandFilterProxyModel;

  void ActivateCurrent();
  void MoveCurrent(int offset);
  void SelectFirstVisible();

  QPointer<QAbstractItemModel> model_;
  CommandFilterProxyModel* proxy_model_ = nullptr;
  SearchBox* search_box_ = nullptr;
  QListView* list_view_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  MatchMode match_mode_ = MatchMode::kFuzzy;
  int id_role_ = Qt::UserRole + 1;
  int data_role_ = Qt::UserRole + 2;
};

}  // namespace autoviz::widget
