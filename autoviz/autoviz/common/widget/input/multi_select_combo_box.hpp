/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVariant>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAction;
class QMenu;
class QToolButton;

namespace autoviz::widget {

struct MultiSelectItem final {
  QString text;
  QVariant data;
  bool enabled = true;
};

// A bounded multi-value selector backed by a native QMenu. It avoids one
// QWidget per row and therefore remains inexpensive for large option lists.
class MultiSelectComboBox final : public QWidget {
  Q_OBJECT

 public:
  explicit MultiSelectComboBox(QWidget* parent = nullptr);

  const QVector<MultiSelectItem>& items() const noexcept { return items_; }
  void SetItems(QVector<MultiSelectItem> items);
  QVector<int> selected_indices() const;
  QVariantList selected_data() const;
  void SetSelectedIndices(const QVector<int>& indices);
  int maximum_selection_count() const noexcept { return maximum_selection_; }
  void SetMaximumSelectionCount(int count);
  QString placeholder_text() const { return placeholder_text_; }
  void SetPlaceholderText(const QString& text);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void SelectionChanged(const QVector<int>& selected_indices);

 private:
  void RebuildMenu();
  void HandleActionToggled(int index, bool checked);
  void UpdatePresentation();

  QVector<MultiSelectItem> items_;
  QVector<QAction*> actions_;
  QToolButton* button_ = nullptr;
  QMenu* menu_ = nullptr;
  QString placeholder_text_;
  style::Tone tone_ = style::Tone::Shell;
  int maximum_selection_ = 0;
  bool updating_actions_ = false;
};

}  // namespace autoviz::widget
