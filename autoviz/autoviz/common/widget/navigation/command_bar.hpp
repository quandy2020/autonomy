/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QToolBar>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

// Lightweight QAction-based command surface. QToolBar provides native
// overflow, shortcut, focus, menu, and accessibility semantics.
class CommandBar final : public QToolBar {
  Q_OBJECT

 public:
  enum class DisplayMode { kIconOnly, kTextBesideIcon, kTextOnly };

  explicit CommandBar(QWidget *parent = nullptr);

  QAction *AddCommand(const QString &text, const QIcon &icon = {});
  DisplayMode display_mode() const noexcept { return display_mode_; }
  void SetDisplayMode(DisplayMode mode);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 private:
  void ApplyTheme();

  DisplayMode display_mode_ = DisplayMode::kTextBesideIcon;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
