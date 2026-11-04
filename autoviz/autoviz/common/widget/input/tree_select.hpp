/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QSet>
#include <QVariant>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAction;
class QMenu;
class QToolButton;

namespace autoviz::widget {

struct TreeSelectItem final {
  QString id;
  QString parent_id;
  QString text;
  QVariant data;
  bool enabled = true;
  bool selectable = true;
};

// Hierarchical single-value selector using native QMenu submenus.
class TreeSelect final : public QWidget {
  Q_OBJECT

 public:
  explicit TreeSelect(QWidget* parent = nullptr);

  void SetItems(const QVector<TreeSelectItem>& items);
  QString current_id() const;
  QVariant current_data() const;
  bool SetCurrentId(const QString& id);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);
  QString placeholder_text() const { return placeholder_text_; }
  void SetPlaceholderText(const QString& text);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void CurrentItemChanged(const QString& id, const QVariant& data);

 private:
  void RebuildMenu();
  void PopulateMenu(QMenu* menu, const QString& parent_id,
                    QSet<QString>* ancestry);
  const TreeSelectItem* FindItemById(const QString& id) const;
  bool IsItemSelectable(const TreeSelectItem* item) const;
  void UpdateCheckedActions();
  void UpdatePresentation();

  QVector<TreeSelectItem> items_;
  QHash<QString, QAction*> actions_;
  QToolButton* button_ = nullptr;
  QMenu* menu_ = nullptr;
  QString current_id_;
  QString placeholder_text_;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
