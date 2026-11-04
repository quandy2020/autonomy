/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QLineEdit>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAction;

namespace autoviz::widget {

// Password input with an explicit, keyboard-accessible reveal action.
class PasswordBox final : public QLineEdit {
 public:
  explicit PasswordBox(QWidget* parent = nullptr);

  bool is_password_visible() const noexcept { return password_visible_; }
  void SetPasswordVisible(bool visible);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;

 protected:
  bool event(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  void UpdateRevealAction();

  QAction* reveal_action_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  bool password_visible_ = false;
};

}  // namespace autoviz::widget
