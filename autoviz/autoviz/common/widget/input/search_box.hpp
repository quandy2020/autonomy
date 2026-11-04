/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QLineEdit>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

// Single-line search input with predictable keyboard and accessibility
// behavior. Return submits the current query and Escape clears it.
class SearchBox final : public QLineEdit {
  Q_OBJECT

 public:
  enum class Size { kCompact, kSmall, kMedium, kLarge };

  explicit SearchBox(QWidget* parent = nullptr);

  Size search_box_size() const noexcept { return size_; }
  void SetSearchBoxSize(Size size);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void SearchRequested(const QString& query);

 protected:
  bool event(QEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  int ControlHeight(const ThemeTokens& tokens) const noexcept;

  style::Tone tone_ = style::Tone::Shell;
  Size size_ = Size::kMedium;
};

}  // namespace autoviz::widget
